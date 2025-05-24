#ifndef OPTIONS_H
#define OPTIONS_H

#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <cstdlib>
#include <algorithm>
#include <cstring>

template <typename T>
struct OptionReader {
    static bool read(const char* arg, T& value) {
        return false;
    }
};

template <>
struct OptionReader<bool> {
    static bool read(const char* arg, bool& value) {
        if (!std::strcmp(arg, "true") || (arg[0] == '1' && arg[1] == '\0') || arg[0] == '\0') {
            value = true;
        } else if (!std::strcmp(arg, "false") || (arg[0] == '0' && arg[1] == '\0')) {
            value = false;
        } else {
            value = false;
            return false;
        }
        return true;
    }
};

template <>
struct OptionReader<std::string> {
    static bool read(const char* arg, std::string& value) {
        value = arg;
        return true;
    }
};

template <>
struct OptionReader<int> {
    static bool read(const char* arg, int& value) {
        char* tmp;
        value = std::strtol(arg, &tmp, 10);
        return tmp != arg;
    }
};

template <>
struct OptionReader<size_t> {
    static bool read(const char* arg, size_t& value) {
        char* tmp;
        value = std::strtoul(arg, &tmp, 10);
        return tmp != arg;
    }
};

template <>
struct OptionReader<float> {
    static bool read(const char* arg, float& value) {
        char* tmp;
        value = std::strtof(arg, &tmp);
        return tmp != arg;
    }
};

template <>
struct OptionReader<double> {
    static bool read(const char* arg, double& value) {
        char* tmp;
        value = std::strtod(arg, &tmp);
        return tmp != arg;
    }
};

template <typename T>
struct OptionWriter {
    static std::string write(const T& value) {
        return std::to_string(value);
    }
};

template <>
struct OptionWriter<std::string> {
    static std::string write(const std::string& value) {
        return value;
    }
};

struct Option {
    Option(const std::string& fn,
           const std::string& sn,
           const std::string& dc)
        : full_name(fn)
        , short_name(sn)
        , desc(dc)
        {}

    virtual ~Option() {}

    virtual std::ostream& print_default(std::ostream& o) const { return o; }
    virtual bool read_value(const char* arg) = 0;
    virtual std::string arg_name() const { return ""; }
    virtual bool has_arg() const { return false; }

    std::string full_name;
    std::string short_name;
    std::string desc;
};

template <typename T>
struct OptionImpl : public Option {
    OptionImpl(const std::string& full_name,
               const std::string& short_name,
               const std::string& desc,
               T& val, const T& def,
               const std::string& arg)
        : Option(full_name, short_name, desc)
        , value(val)
        , default_value(def)
        , arg_desc(arg)
    {
        value = default_value;
    }

    std::ostream& print_default(std::ostream& o) const override { return o << default_value; }
    
    bool read_value(const char* arg) override {
        return OptionReader<T>::read(arg, value);
    }

    bool has_arg() const override { return true; }
    std::string arg_name() const override { return arg_desc; }

    T& value;
    T default_value;
    std::string arg_desc;
};

template <>
struct OptionImpl<bool> : public Option {
    OptionImpl(const std::string& full_name,
               const std::string& short_name,
               const std::string& desc,
               bool& val, const bool& def,
               const std::string&)
        : Option(full_name, short_name, desc)
        , value(val)
        , default_value(def)
    {
        value = default_value;
    }

    bool read_value(const char* arg) override {
        return OptionReader<bool>::read(arg, value);
    }

    bool& value;
    bool default_value;
};

/// Command line argument parser with ability to display the program usage.
class ArgParser {
public:
    ArgParser(int argc, char** argv)
        : argc(argc), argv(argv)
    {
        std::string path(argv[0]);
        exe_name = path.substr(path.find_last_of("\\/") + 1);
    }

    ~ArgParser() {
        for (auto opt : options) {
            delete opt;
        }
    }

    template <typename T>
    void add_option(const std::string& full_name,
                    const std::string& short_name,
                    const std::string& desc,
                    T& value,
                    const T& default_value = T(),
                    const std::string& arg_name = std::string()) {
        options.push_back(new OptionImpl<T>(full_name, short_name, desc, value, default_value, arg_name));
    }

    bool parse() {
        for (int i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                // This is an option
                if (argv[i][1] == '-') {
                    // Try full name
                    const std::string full_opt(argv[i] + 2);
                    auto eq_pos = full_opt.find('=');
                    const std::string opt_name = full_opt.substr(0, eq_pos);
                    const std::string opt_arg = (eq_pos != std::string::npos) ? full_opt.substr(eq_pos + 1) : std::string();

                    auto it = std::find_if(options.begin(), options.end(), [this, opt_name] (const Option* opt) -> bool {
                        return opt->full_name == opt_name;
                    });

                    if (it == options.end()) {
                        std::cerr << "Unknown option : " << opt_name << std::endl;
                        return false;
                    }

                    if ((*it)->has_arg()) {
                        if (eq_pos == std::string::npos) {
                            std::cerr << "Missing argument for option : " << opt_name << std::endl;
                            return false;
                        }

                        if (!(*it)->read_value(opt_arg.c_str())) {
                            std::cerr << "Invalid value given to option \'" << opt_name << "\' : " << opt_arg << std::endl;
                            return false;
                        }
                    } else {
                        if (eq_pos != std::string::npos) {
                            std::cerr << "Option \'" << opt_name << "\' does not accept any argument" << std::endl;
                            return false;
                        }

                        bool ok = (*it)->read_value(opt_arg.c_str());
                        UNUSED(ok);
                        assert(ok && "read_value returns false for an option without args");
                    }
                } else {
                    // Try short name
                    auto it = std::find_if(options.begin(), options.end(), [this, i] (const Option* opt) -> bool {
                        return opt->short_name.compare(argv[i] + 1) == 0;
                    });

                    if (it == options.end()) {
                        std::cerr << "Unknown option : " << argv[i] + 1 << std::endl;
                        return false;
                    }

                    if ((*it)->has_arg()) {
                        if (i >= argc - 1) {
                            std::cerr << "Missing argument for option : " << argv[i] + 1 << std::endl;
                            return false;
                        }

                        if(!(*it)->read_value(argv[i + 1])) {
                            std::cerr << "Invalid value given to option \'" << argv[i] + 1 << "\' : " << argv[i + 1] << std::endl;
                            return false;
                        }

                        i++;
                    } else {
                        if(!(*it)->read_value("")) {
                            std::cerr << "Invalid value given to option \'" << argv[i] + 1 << std::endl;
                            return false;
                        }
                    }
                }
            } else {
                // This is an argument
                args.push_back(argv[i]);
            }
        }
        return true;
    }

    void usage() const {
        std::cout << "Usage : " << exe_name << " [options] files\n"
                  << "Available options :\n";
        
        // Compute amount of space needed to align text
        size_t short_offset = 0, full_offset = 0;
        for (const auto opt : options) {
            auto a = opt->has_arg() ? opt->arg_name().length() + 2 : 0;
            auto s = opt->short_name.length() + a;
            auto f = opt->full_name.length() + a;

            if (short_offset < s) short_offset = s;
            if (full_offset < f) full_offset = f;
        }

        for (const auto opt : options) {
            auto s = opt->short_name.length();
            auto f = opt->full_name.length();

            if (opt->has_arg()) {
                auto a = opt->arg_name().length();
                std::cout << "\t-" << opt->short_name << " " << opt->arg_name() << std::string(short_offset - s - a, ' ')
                          << "--" << opt->full_name << "=" << opt->arg_name() << std::string(full_offset - f - a, ' ')
                          << opt->desc << " (defaults to \'";
                opt->print_default(std::cout) << "\').\n";
            } else {
                std::cout << "\t-" << opt->short_name << std::string(short_offset - s + 1, ' ')
                          << "--" << opt->full_name << std::string(full_offset - f + 1, ' ')
                          << opt->desc << ".\n";
            }
        }

        std::cout << std::endl;
    }

    const std::vector<std::string>& arguments() const {
        return args;
    }

private:
    std::vector<Option*> options;
    std::vector<std::string> args;
    std::string exe_name;
    int argc;
    char** argv;
};

#endif // OPTIONS_H
