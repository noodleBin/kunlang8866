#ifndef CENTURY_PERCEPTION_SINGLETON_H_
#define CENTURY_PERCEPTION_SINGLETON_H_

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
using namespace std;

namespace century {
namespace perception {
namespace base {

template <typename T>
class Singleton {
 public:
  template <typename... Args>
  static T* Instance(Args&&... args) {
    if (m_pInstance == nullptr)
      m_pInstance = new T(std::forward<Args>(args)...);

    return m_pInstance;
  }

  static T* GetInstance() {
    if (m_pInstance == nullptr)
      throw std::logic_error(
          "the instance is not init, please initialize the instance first");
    return m_pInstance;
  }
  static void DestroyInstance() {
    delete m_pInstance;
    m_pInstance = nullptr;
  }

 private:
  Singleton(void);
  virtual ~Singleton(void);
  Singleton(const Singleton&);
  Singleton& operator=(const Singleton&);

 private:
  static T* m_pInstance;
};

template <class T>
T* Singleton<T>::m_pInstance = nullptr;

}  // namespace base
}  // namespace perception
}  // namespace century

#endif