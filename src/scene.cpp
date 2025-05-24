#include <chrono>
#include <fstream>
#include <cassert>

#include <yaml-cpp/yaml.h>

#include "scene.h"
#include "load_obj.h"

namespace YAML {
    static std::ostream& operator << (std::ostream& os, const YAML::Mark& mark) {
        if (mark.line < 0 && mark.column < 0) return os;
        assert(mark.line >= 0);
        os << "(line " << mark.line + 1;
        if (mark.column >= 0)
            os << ", column " << mark.column + 1;
        os << ")";
        return os;
    }
}

struct TriIdx {
    int v0, v1, v2, m;
    TriIdx(int v0, int v1, int v2, int m)
        : v0(v0), v1(v1), v2(v2), m(m)
    {}
};

struct HashIndex {
    size_t operator () (const obj::Index& i) const {
        unsigned h = 0, g;

        h = (h << 4) + i.v;
        g = h & 0xF0000000;
        h = g ? (h ^ (g >> 24)) : h;
        h &= ~g;

        h = (h << 4) + i.t;
        g = h & 0xF0000000;
        h = g ? (h ^ (g >> 24)) : h;
        h &= ~g;

        h = (h << 4) + i.n;
        g = h & 0xF0000000;
        h = g ? (h ^ (g >> 24)) : h;
        h &= ~g;

        return h;
    }
};

struct CompareIndex {
    bool operator () (const obj::Index& a, const obj::Index& b) const {
        return a.v == b.v && a.t == b.t && a.n == b.n;
    }
};

typedef std::unordered_map<std::string, int> TextureMap;

static void compute_face_normals(const std::vector<uint32_t>& indices,
                                 const std::vector<float3>& vertices,
                                 std::vector<float3>& face_normals,
                                 size_t first_index) {
    for (auto i = first_index, k = indices.size(); i < k; i += 4) {
        const float3& v0 = vertices[indices[i + 0]];
        const float3& v1 = vertices[indices[i + 1]];
        const float3& v2 = vertices[indices[i + 2]];
        face_normals[i / 4] = normalize(cross(v1 - v0, v2 - v0));
    }
}

static void recompute_normals(const std::vector<uint32_t>& indices,
                              const std::vector<float3>& face_normals,
                              std::vector<float3>& normals,
                              size_t first_index) {
    for (auto i = first_index, k = indices.size(); i < k; i += 4) {
        float3& n0 = normals[indices[i + 0]];
        float3& n1 = normals[indices[i + 1]];
        float3& n2 = normals[indices[i + 2]];
        const float3& n = face_normals[i / 4];
        n0 += n;
        n1 += n;
        n2 += n;
    }
}

static int load_texture(const FilePath& path, TextureMap& tex_map, Scene& scene) {
    auto it = tex_map.find(path);
    if (it != tex_map.end())
        return it->second;

    int id = -1;

    Image img;
    if (load_png(path, img) || load_tga(path, img) || load_jpeg(path, img) || load_tiff(path, img) || load_exr(path, img)) {
        id = scene.textures.size();
        assert(img.width * img.height > 0);
        scene.textures.emplace_back(new ImageTexture(std::move(img)));
    } else {
        warn("Invalid PNG/TGA/JPEG/TIFF/EXR texture '", path.path(), "'.");
    }

    tex_map[path] = id;
    return id;
}

static bool load_mesh(const std::string& file, TextureMap& tex_map, Scene& scene) {
    FilePath path(file);

    obj::File obj_file;
    if (!load_obj(path, obj_file)) {
        error("Cannot open OBJ file '", file, "'.");
        return false;
    }

    obj::MaterialLib mat_lib;
    for (auto& lib_file : obj_file.mtl_libs) {
        if (!load_mtl(path.base_name() + "/" + lib_file, mat_lib)) {
            error("Cannot open MTL file '", lib_file, "'.");
            return false;
        }
    }

    const int mtl_offset = scene.materials.size();

    // Create a dummy constant texture color for incorrect texture references
    auto dummy_tex  = new ConstantTexture(rgb(1.0f, 0.0f, 1.0f));
    auto dummy_bsdf = new DiffuseBsdf(*dummy_tex);

    // Create one material for objects without materials
    scene.textures.emplace_back(dummy_tex);
    scene.bsdfs.emplace_back(dummy_bsdf);
    scene.materials.emplace_back(dummy_bsdf);

    std::vector<rgb> map_ke(obj_file.materials.size(), rgb(0.0f));

    // Create the materials for this OBJ file
    for (int i = 1, n = obj_file.materials.size(); i < n; i++) {
        auto it = mat_lib.find(obj_file.materials[i]);
        if (it == mat_lib.end()) {
            warn("Cannot find material '", obj_file.materials[i], "'.");
            scene.materials.emplace_back(scene.bsdfs[0].get());
            continue;
        }

        const obj::Material& mat = it->second;

        Bsdf* bsdf = nullptr;
        map_ke[i]  = mat.ke;

        switch (mat.illum) {
            case 5: bsdf = new MirrorBsdf(mat.ks); break;
            case 7: bsdf = new GlassBsdf(1.0f, mat.ni, mat.ks, mat.tf); break;
            default:
                Texture* diff_tex = nullptr;
                if (mat.map_kd != "") {
                    int id = load_texture(path.base_name() + "/" + mat.map_kd, tex_map, scene);
                    diff_tex = id >= 0 ? scene.textures[id].get() : nullptr;
                }

                Texture* spec_tex = nullptr;
                if (mat.map_ks != "") {
                    int id = load_texture(path.base_name() + "/" + mat.map_ks, tex_map, scene);
                    spec_tex = id >= 0 ? scene.textures[id].get() : nullptr;
                }

                auto kd = dot(mat.kd, luminance);
                auto ks = dot(mat.ks, luminance);
                Bsdf* diff = nullptr, *spec = nullptr;

                if (ks > 0 || spec_tex) {
                    if (!spec_tex) {
                        scene.textures.emplace_back(new ConstantTexture(mat.ks));
                        spec_tex = scene.textures.back().get();
                    }
                    spec = new GlossyPhongBsdf(*spec_tex, mat.ns);
                    ks = ks == 0 ? 1.0f : ks;
                }

                if (kd > 0 || diff_tex) {
                    if (!diff_tex) {
                        scene.textures.emplace_back(new ConstantTexture(mat.kd));
                        diff_tex = scene.textures.back().get();
                    }
                    diff = new DiffuseBsdf(*diff_tex);
                    kd = kd == 0 ? 1.0f : kd;
                }

                if (spec && diff) {
                    auto k  = ks / (kd + ks);
                    auto ty = k < 0.2f || mat.ns < 10.0f // Approximate threshold
                        ? Bsdf::Type::Diffuse
                        : Bsdf::Type::Glossy;
                    bsdf = new CombineBsdf(ty, diff, spec, k);
                } else {
                    bsdf = diff ? diff : spec;
                }

                break;
        }
        scene.bsdfs.emplace_back(bsdf);
        scene.materials.emplace_back(bsdf);
    }

    for (auto& obj: obj_file.objects) {
        // Convert the faces to triangles & build the new list of indices
        std::vector<TriIdx> triangles;
        std::unordered_map<obj::Index, int, HashIndex, CompareIndex> mapping;

        bool has_normals = false;
        bool has_texcoords = false;
        for (auto& group : obj.groups) {
            for (auto& face : group.faces) {
                for (size_t i = 0; i < face.indices.size(); i++) {
                    auto map = mapping.find(face.indices[i]);
                    if (map == mapping.end()) {
                        has_normals |= (face.indices[i].n != 0);
                        has_texcoords |= (face.indices[i].t != 0);

                        mapping.insert(std::make_pair(face.indices[i], mapping.size()));
                    }
                }

                const int mtl_idx = face.material + mtl_offset;
                const int v0 = mapping[face.indices[0]];
                int prev = mapping[face.indices[1]];

                for (size_t i = 1; i < face.indices.size() - 1; i++) {
                    const int next = mapping[face.indices[i + 1]];

                    int new_mtl_idx = mtl_idx;
                    auto& ke = map_ke[mtl_idx - mtl_offset];
                    if (lensqr(ke) > 0.0f) {
                        // This triangle is a light
                        scene.lights.emplace_back(
                            new TriangleLight(obj_file.vertices[face.indices[0 + 0].v],
                                              obj_file.vertices[face.indices[i + 0].v],
                                              obj_file.vertices[face.indices[i + 1].v],
                                              ke));
                        new_mtl_idx = scene.materials.size();
                        scene.materials.emplace_back(
                            scene.materials[mtl_idx].bsdf,
                            scene.lights.back().get());
                    }
                    triangles.emplace_back(v0, prev, next, new_mtl_idx);
                    prev = next;
                }
            }
        }

        if (triangles.size() == 0) continue;

        // Add this object to the scene
        const int vtx_offset = scene.vertices.size();
        const int idx_offset = scene.indices.size();
        scene.indices.resize(idx_offset + 4 * triangles.size());
        scene.vertices.resize(vtx_offset + mapping.size());
        scene.texcoords.resize(vtx_offset + mapping.size());
        scene.normals.resize(vtx_offset + mapping.size());

        for (int i = 0, n = triangles.size(); i < n; i++) {
            auto& t = triangles[i];
            scene.indices[idx_offset + i * 4 + 0] = t.v0 + vtx_offset;
            scene.indices[idx_offset + i * 4 + 1] = t.v1 + vtx_offset;
            scene.indices[idx_offset + i * 4 + 2] = t.v2 + vtx_offset;
            scene.indices[idx_offset + i * 4 + 3] = t.m;
        }

        for (auto& p : mapping) {
            const auto& v = obj_file.vertices[p.first.v];
            scene.vertices[vtx_offset + p.second].x = v.x;
            scene.vertices[vtx_offset + p.second].y = v.y;
            scene.vertices[vtx_offset + p.second].z = v.z;
        }

        if (has_texcoords) {
            for (auto& p : mapping) {
                const auto& t = obj_file.texcoords[p.first.t];
                scene.texcoords[vtx_offset + p.second] = t;
            }
        } else std::fill(scene.texcoords.begin() + vtx_offset, scene.texcoords.end(), float2(0.0f));

        // Compute the geometric normals for this mesh
        scene.face_normals.resize(scene.face_normals.size() + triangles.size());
        compute_face_normals(scene.indices, scene.vertices, scene.face_normals, idx_offset);

        if (has_normals) {
            // Set up mesh normals
            for (auto& p : mapping) {
                const auto& n = obj_file.normals[p.first.n];
                scene.normals[vtx_offset + p.second] = n;
            }
        } else {
            // Recompute normals
            warn("No normals are present, recomputing smooth normals from geometry.");
            std::fill(scene.normals.begin() + vtx_offset, scene.normals.end(), float3(0.0f));
            recompute_normals(scene.indices, scene.face_normals, scene.normals, idx_offset);
        }
    }

    // Re-normalize all the values in the OBJ file to handle invalid meshes
    for (auto& n : scene.normals) {
        auto len2 = lensqr(n);
        if (len2 == 0.0f || std::isnan(len2))
            n = float3(0.0f, 1.0f, 0.0f);
        else
            n *= 1.0f / std::sqrt(len2);
    }

    return true;
}

static float3 parse_float3(const YAML::Node& node) {
    return float3(node[0].as<float>(), node[1].as<float>(), node[2].as<float>());
}

static void setup_camera(Scene& scene, const YAML::Node& node) {
    if (node.Tag() == "!perspective_camera") {
        scene.camera.reset(new PerspectiveCamera(
            parse_float3(node["eye"]),
            parse_float3(node["center"]),
            parse_float3(node["up"]),
            node["fov"].as<float>(),
            float(scene.width) / float(scene.height)));
    } else {
        throw YAML::Exception(node.Mark(), "unknown camera type");
    }
}

static void setup_light(Scene& scene, const YAML::Node& node) {
    if (node.Tag() == "!point_light") {
        scene.lights.emplace_back(new PointLight(
            parse_float3(node["position"]),
            parse_float3(node["color"])));
    } else if (node.Tag() == "!triangle_light") {
        uint32_t first = scene.vertices.size();
        auto v0 = parse_float3(node["v0"]);
        auto v1 = parse_float3(node["v1"]);
        auto v2 = parse_float3(node["v2"]);
        auto n  = cross(v1 - v0, v2 - v0);
        scene.vertices.emplace_back(v0);
        scene.vertices.emplace_back(v1);
        scene.vertices.emplace_back(v2);
        scene.texcoords.emplace_back(0.0f);
        scene.texcoords.emplace_back(0.0f);
        scene.normals.emplace_back(n);
        scene.normals.emplace_back(n);
        scene.normals.emplace_back(n);
        scene.face_normals.emplace_back(n);
        auto color = parse_float3(node["color"]);
        scene.lights.emplace_back(new TriangleLight(
            scene.vertices[first + 0],
            scene.vertices[first + 1],
            scene.vertices[first + 2],
            color));
        uint32_t mat = scene.materials.size();
        scene.indices.insert(scene.indices.end(),
            {first, first + 1, first + 2, mat});
        scene.materials.emplace_back(nullptr, scene.lights.back().get());
    } else {
        throw YAML::Exception(node.Mark(), "unknown light type");
    }
}

bool validate_scene(const Scene& scene) {
    if (scene.vertices.size() == 0) {
        error("There is no mesh in the scene.");
        return false;
    }

    if (scene.lights.size() == 0) {
        error("There are no lights in the scene.");
        return false;
    }

    if (!scene.camera) {
        error("There is no camera in the scene.");
        return false;
    }

    return true;
}

bool load_scene(const std::string& config, Scene& scene) {
    using namespace std::chrono;

    if (!std::ifstream(config)) {
        error("The scene file '", config, "' cannot be opened.");
        return false;
    }

    auto start_load = high_resolution_clock::now();
    try {
        auto node = YAML::LoadFile(config);
        TextureMap tex_map;
        FilePath config_path(config);
        for (const auto& mesh : node["meshes"]) load_mesh(config_path.base_name() + "/" + mesh.as<std::string>(), tex_map, scene);
        for (const auto& light : node["lights"]) setup_light(scene, light);
        setup_camera(scene, node["camera"]);
    } catch (YAML::Exception& e) {
        error("Configuration error: ", e.msg, " ", e.mark);
        return false;
    }
    auto end_load = high_resolution_clock::now();

    if (!validate_scene(scene)) return false;

    int num_verts = scene.vertices.size();
    int num_tris  = scene.indices.size() / 4;
    info("Scene loaded in ", duration_cast<milliseconds>(end_load - start_load).count(), " ms (",
         num_verts, " vertices, ", num_tris, " triangles).");

    // Build BVH
    auto start_bvh = high_resolution_clock::now();
    scene.bvh.build(scene.vertices.data(), scene.indices.data(), num_tris);
    auto end_bvh = high_resolution_clock::now();
    info("BVH constructed in ", duration_cast<milliseconds>(end_bvh - start_bvh).count(), " ms (",
         scene.bvh.node_count(), " nodes).");

    return true;
}
