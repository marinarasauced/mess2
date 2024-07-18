#ifndef MESS2_PLUGINS__VISIBILITY_CONTROL_H_
#define MESS2_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MESS2_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define MESS2_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define MESS2_PLUGINS_EXPORT __declspec(dllexport)
    #define MESS2_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MESS2_PLUGINS_BUILDING_LIBRARY
    #define MESS2_PLUGINS_PUBLIC MESS2_PLUGINS_EXPORT
  #else
    #define MESS2_PLUGINS_PUBLIC MESS2_PLUGINS_IMPORT
  #endif
  #define MESS2_PLUGINS_PUBLIC_TYPE MESS2_PLUGINS_PUBLIC
  #define MESS2_PLUGINS_LOCAL
#else
  #define MESS2_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define MESS2_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define MESS2_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define MESS2_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MESS2_PLUGINS_PUBLIC
    #define MESS2_PLUGINS_LOCAL
  #endif
  #define MESS2_PLUGINS_PUBLIC_TYPE
#endif

#endif  // MESS2_PLUGINS__VISIBILITY_CONTROL_H_
