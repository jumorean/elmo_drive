#ifndef IIT_PEGASUS2_LINK_DATA_MAP_H_
#define IIT_PEGASUS2_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace pegasus2 {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[TORSO] = rhs[TORSO];
    data[RF_HIP] = rhs[RF_HIP];
    data[RF_UPPERLEG] = rhs[RF_UPPERLEG];
    data[RF_LOWERLEG] = rhs[RF_LOWERLEG];
    data[RH_HIP] = rhs[RH_HIP];
    data[RH_UPPERLEG] = rhs[RH_UPPERLEG];
    data[RH_LOWERLEG] = rhs[RH_LOWERLEG];
    data[LH_HIP] = rhs[LH_HIP];
    data[LH_UPPERLEG] = rhs[LH_UPPERLEG];
    data[LH_LOWERLEG] = rhs[LH_LOWERLEG];
    data[LF_HIP] = rhs[LF_HIP];
    data[LF_UPPERLEG] = rhs[LF_UPPERLEG];
    data[LF_LOWERLEG] = rhs[LF_LOWERLEG];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[TORSO] = value;
    data[RF_HIP] = value;
    data[RF_UPPERLEG] = value;
    data[RF_LOWERLEG] = value;
    data[RH_HIP] = value;
    data[RH_UPPERLEG] = value;
    data[RH_LOWERLEG] = value;
    data[LH_HIP] = value;
    data[LH_UPPERLEG] = value;
    data[LH_LOWERLEG] = value;
    data[LF_HIP] = value;
    data[LF_UPPERLEG] = value;
    data[LF_LOWERLEG] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   torso = "
    << map[TORSO]
    << "   RF_hip = "
    << map[RF_HIP]
    << "   RF_upperleg = "
    << map[RF_UPPERLEG]
    << "   RF_lowerleg = "
    << map[RF_LOWERLEG]
    << "   RH_hip = "
    << map[RH_HIP]
    << "   RH_upperleg = "
    << map[RH_UPPERLEG]
    << "   RH_lowerleg = "
    << map[RH_LOWERLEG]
    << "   LH_hip = "
    << map[LH_HIP]
    << "   LH_upperleg = "
    << map[LH_UPPERLEG]
    << "   LH_lowerleg = "
    << map[LH_LOWERLEG]
    << "   LF_hip = "
    << map[LF_HIP]
    << "   LF_upperleg = "
    << map[LF_UPPERLEG]
    << "   LF_lowerleg = "
    << map[LF_LOWERLEG]
    ;
    return out;
}

}
}
#endif
