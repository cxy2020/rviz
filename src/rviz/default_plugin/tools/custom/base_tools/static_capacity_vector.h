#ifndef TRAJ_TOOLS_COMMON_CAPACITY_VECTOR_H_
#define TRAJ_TOOLS_COMMON_CAPACITY_VECTOR_H_

#include <stddef.h>

namespace traj_tools {
namespace common {

template<class T>
class StaticCapacityVector {
public:
    class Iterator {
    public:
        inline Iterator(const StaticCapacityVector* vector);

        inline Iterator(int index, const StaticCapacityVector* vector);

        inline const Iterator& operator++();

        inline const Iterator& operator--();

        inline bool operator==(const Iterator& iter) const;

        inline bool operator!=(const Iterator& iter) const;

        inline const T& operator*() const;

        inline T& operator*();

    public:
        int m_iter_index;
        const StaticCapacityVector* m_vector;
    };

    StaticCapacityVector(int size);

    ~StaticCapacityVector();

    inline void clear();

    inline bool push_back(const T& data);

    inline void pop_back();

    inline const T& operator[](int index) const;

    inline Iterator begin() const;

    inline Iterator end() const;

    inline const T& front() const;

    inline const T& back() const;

    inline int size() const {
        return (m_index + 1);
    }

    inline int capacity() const {
        return m_capacity;
    }

private:
    friend class Iterator;

    inline bool is_end(int index) const;

    inline int end_index() const;

    T* m_data;
    int m_capacity;
    int m_index;
};

template<class T>
StaticCapacityVector<T>::StaticCapacityVector(int size) :
    m_data(new T[size + 1]),
    m_capacity(size),
    m_index(-1) {}

template<class T>
StaticCapacityVector<T>::~StaticCapacityVector() {
    delete m_data;
}

template<class T>
void StaticCapacityVector<T>::clear() {
    m_index = -1;
}

template<class T>
bool StaticCapacityVector<T>::push_back(const T& data) {
    if(++m_index == m_capacity) {
        --m_index;
        return false;
    }
    m_data[m_index] = data;
    return true;
}

template<class T>
void StaticCapacityVector<T>::pop_back() {
    if(m_index < 0) {
        return;
    }
    if(--m_index < 0) {
        m_index = 0;
    }
}

template<class T>
const T& StaticCapacityVector<T>::operator[](int index) const {
    if(index <= m_index) {
        return m_data[index];
    }
    return m_data[m_capacity];
}

template<class T>
typename StaticCapacityVector<T>::Iterator StaticCapacityVector<T>::begin() const {
    if(m_index >= 0) {
        return StaticCapacityVector<T>::Iterator(0, this);
    }
    return StaticCapacityVector<T>::Iterator(m_capacity, this);
}

template<class T>
typename StaticCapacityVector<T>::Iterator StaticCapacityVector<T>::end() const {
    return StaticCapacityVector<T>::Iterator(m_capacity, this);
}

template<class T>
const T& StaticCapacityVector<T>::front() const {
    return m_data[0];
}

template<class T>
const T& StaticCapacityVector<T>::back() const {
    return m_data[m_index];
}

template<class T>
bool StaticCapacityVector<T>::is_end(int index) const {
    return (index == m_capacity);
}

template<class T>
int StaticCapacityVector<T>::end_index() const {
    return m_capacity;
}

template<class T>
StaticCapacityVector<T>::Iterator::Iterator(const StaticCapacityVector* vector) :
    m_iter_index(vector->end_index()), m_vector(vector) {}

template<class T>
StaticCapacityVector<T>::Iterator::Iterator(int index, const StaticCapacityVector* vector) :
    m_iter_index(index),
    m_vector(vector) {}

template<class T>
const typename StaticCapacityVector<T>::Iterator& StaticCapacityVector<T>::Iterator::operator++() {
    if(!m_vector->is_end(m_iter_index)) {
        if(++m_iter_index > m_vector->m_index) {
            m_iter_index = m_vector->end_index();
        }
        return *this;
    }
    return *this;
}

template<class T>
const typename StaticCapacityVector<T>::Iterator& StaticCapacityVector<T>::Iterator::operator--() {
    if(m_vector->is_end(m_iter_index)) {
        m_iter_index = m_vector->m_index;
        return *this;
    }
    if(--m_iter_index >= 0) {
        return *this;
    }
    ++m_iter_index;
    return *this;
}

template<class T>
bool StaticCapacityVector<T>::Iterator::operator==(const StaticCapacityVector::Iterator& iter) const {
    if(m_iter_index == iter.m_iter_index && m_vector == iter.m_vector) {
        return true;
    }
    return false;
}

template<class T>
bool StaticCapacityVector<T>::Iterator::operator!=(const StaticCapacityVector::Iterator& iter) const {
    if(m_iter_index == iter.m_iter_index && m_vector == iter.m_vector) {
        return false;
    }
    return true;
}

template<class T>
const T& StaticCapacityVector<T>::Iterator::operator*() const {
    return m_vector->m_data[m_iter_index];
}

template<class T>
T& StaticCapacityVector<T>::Iterator::operator*() {
    return const_cast<StaticCapacityVector<T>*>(m_vector)->m_data[m_iter_index];
}
}       //namespace common
}       //namespace traj_tools

#endif // TRAJ_TOOLS_COMMON_CAPACITY_VECTOR_H_
