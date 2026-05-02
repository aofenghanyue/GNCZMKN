#pragma once

#include <map>
#include <string>
#include <vector>

namespace gnc::perturbation {

struct PerturbationValue {
    enum class Type {
        Number,
        String,
        Vector
    };

    Type type = Type::Number;
    double number = 0.0;
    std::string string;
    std::vector<double> vector;
};

class IPerturbationProvider {
public:
    virtual ~IPerturbationProvider() = default;

    virtual bool has(const std::string& key) const = 0;
    virtual double getNumber(const std::string& key, double fallback) const = 0;
    virtual std::string getString(const std::string& key,
                                  const std::string& fallback) const = 0;
    virtual std::vector<double> getVector(const std::string& key) const = 0;
};

class IPerturbationSnapshot {
public:
    virtual ~IPerturbationSnapshot() = default;
    virtual std::map<std::string, PerturbationValue> snapshotResolvedState() const = 0;
};

} // namespace gnc::perturbation
