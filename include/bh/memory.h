//==================================================
// memory.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 15, 2017
//==================================================
#pragma once

#include <memory>
#include <functional>

namespace bh {

template <typename KeyT, typename ValueT, typename HashT = std::hash<KeyT>>
class CacheStorage {
public:
  using FactoryType = std::function<std::shared_ptr<ValueT>(const KeyT&)>;
  CacheStorage();

  CacheStorage(const FactoryType factory);

  void setFactory(const FactoryType factory);

  void clear();

  void cleanup();

  size_t size() const;

  bool hasInstance(const KeyT& key) const;

  std::shared_ptr<ValueT> getInstance(const KeyT& key, const bool do_auto_cleanup = true);

private:
  std::unordered_map<KeyT, std::weak_ptr<ValueT>, HashT> storage_;
  FactoryType factory_;
};

// -------------------------
// Hash function for pairs and tuples
// -------------------------

template <typename T,
  typename std::enable_if<std::is_same<T, std::pair<typename T::first_type, typename T::second_type>>::value>::type* = nullptr>
struct hash {
  std::size_t operator()(const T& v) const {
    using std::hash;
    size_t val { 0 };
    boost::hash_combine(val, hash<typename T::first_type>()(v.first));
    boost::hash_combine(val, hash<typename T::second_type>()(v.second));
    return val;
  }
};

template <typename T, typename U>
struct pair_hash {
  std::size_t operator()(const std::pair<T, U>& v) const {
    using std::hash;
    size_t val { 0 };
    boost::hash_combine(val, hash<T>()(v.first));
    boost::hash_combine(val, hash<U>()(v.second));
    return val;
  }
};

#define BH_PAIR_DEFINE_STD_HASH_SPECIALIZATION(T, U)       \
namespace std {                                            \
template<>                                                 \
struct hash<T> {                                           \
  std::size_t operator()(const std::pair<T, U>& v) const { \
    return pair_hash()(v);                                   \
  }                                                        \
};                                                         \
}

// -------------------------
// Pointer utilities
// -------------------------

template<typename T, typename U>
std::unique_ptr<T> static_cast_ptr(std::unique_ptr<U>&& ptr);

template<typename T, typename U>
std::unique_ptr<T> dynamic_cast_ptr(std::unique_ptr<U>&& ptr);

template<typename T, typename U>
std::unique_ptr<T> reinterpret_cast_ptr(std::unique_ptr<U>&& ptr);

// -------------------------
// Pointer utilities implementation
// -------------------------

template<typename T, typename U>
std::unique_ptr<T> static_cast_ptr(std::unique_ptr<U>&& ptr) {
  T* raw_ptr = static_cast<T*>(ptr.release());
  return std::move(std::unique_ptr<T>(raw_ptr));
}

template<typename T, typename U>
std::unique_ptr<T> dynamic_cast_ptr(std::unique_ptr<U>&& ptr) {
  T* raw_ptr = dynamic_cast<T*>(ptr.release());
  return std::move(std::unique_ptr<T>(raw_ptr));
}

template<typename T, typename U>
std::unique_ptr<T> reinterpret_cast_ptr(std::unique_ptr<U>&& ptr) {
  T* raw_ptr = reinterpret_cast<T*>(ptr.release());
  return std::move(std::unique_ptr<T>(raw_ptr));
}

// -------------------------
// Cache storage implementation
// -------------------------

template <typename KeyT, typename ValueT, typename HashT>
CacheStorage<KeyT, ValueT, HashT>::CacheStorage() {
  factory_ = [](const KeyT key) {
    return std::make_shared<ValueT>();
  };
}

template <typename KeyT, typename ValueT, typename HashT>
CacheStorage<KeyT, ValueT, HashT>::CacheStorage(
    const FactoryType factory)
    : factory_(factory) {}

template <typename KeyT, typename ValueT, typename HashT>
void
CacheStorage<KeyT, ValueT, HashT>::setFactory(const FactoryType factory) {
  factory_ = factory;
}

template <typename KeyT, typename ValueT, typename HashT>
void
CacheStorage<KeyT, ValueT, HashT>::clear() {
  storage_.clear();
}

template <typename KeyT, typename ValueT, typename HashT>
void
CacheStorage<KeyT, ValueT, HashT>::cleanup() {
  storage_.erase(std::remove_if(storage_.begin(),
                                storage_.end(),
                                [] (const std::weak_ptr<ValueT>& weak_ptr) { return !weak_ptr; }),
                 storage_.end());
}

template <typename KeyT, typename ValueT, typename HashT>
size_t
CacheStorage<KeyT, ValueT, HashT>::size() const {
  return storage_.size();
}

template <typename KeyT, typename ValueT, typename HashT>
bool
CacheStorage<KeyT, ValueT, HashT>::hasInstance(const KeyT& key) const {
  return storage_.count(key) > 0;
}

template <typename KeyT, typename ValueT, typename HashT>
std::shared_ptr<ValueT>
CacheStorage<KeyT, ValueT, HashT>::getInstance(const KeyT& key, const bool do_auto_cleanup) {
  auto it = storage_.find(key);
  if (it != storage_.end()) {
    std::weak_ptr<ValueT> weak_ptr = it->second;
    std::shared_ptr<ValueT> shared_ptr = weak_ptr.lock();
    if (shared_ptr) {
      return shared_ptr;
    }
  }
  else {
    if (do_auto_cleanup) {
      cleanup();
    }
    std::tie(it, std::ignore) = storage_.emplace(key, std::weak_ptr<ValueT>());
  }
  std::shared_ptr<ValueT> shared_ptr = factory_(key);
  std::weak_ptr<ValueT> weak_ptr(shared_ptr);
  it->second = weak_ptr;
  return shared_ptr;
}

}
