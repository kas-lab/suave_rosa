#ifndef SUAVE_ROSA_BT__VISIBILITY_CONTROL_H_
#define SUAVE_ROSA_BT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SUAVE_ROSA_BT_EXPORT __attribute__ ((dllexport))
    #define SUAVE_ROSA_BT_IMPORT __attribute__ ((dllimport))
  #else
    #define SUAVE_ROSA_BT_EXPORT __declspec(dllexport)
    #define SUAVE_ROSA_BT_IMPORT __declspec(dllimport)
  #endif
  #ifdef SUAVE_ROSA_BT_BUILDING_LIBRARY
    #define SUAVE_ROSA_BT_PUBLIC SUAVE_ROSA_BT_EXPORT
  #else
    #define SUAVE_ROSA_BT_PUBLIC SUAVE_ROSA_BT_IMPORT
  #endif
  #define SUAVE_ROSA_BT_PUBLIC_TYPE SUAVE_ROSA_BT_PUBLIC
  #define SUAVE_ROSA_BT_LOCAL
#else
  #define SUAVE_ROSA_BT_EXPORT __attribute__ ((visibility("default")))
  #define SUAVE_ROSA_BT_IMPORT
  #if __GNUC__ >= 4
    #define SUAVE_ROSA_BT_PUBLIC __attribute__ ((visibility("default")))
    #define SUAVE_ROSA_BT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SUAVE_ROSA_BT_PUBLIC
    #define SUAVE_ROSA_BT_LOCAL
  #endif
  #define SUAVE_ROSA_BT_PUBLIC_TYPE
#endif

#endif  // SUAVE_ROSA_BT__VISIBILITY_CONTROL_H_
