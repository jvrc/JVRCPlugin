#ifndef CNOID_JVRCPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_JVRCPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_JVRCPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_JVRCPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_JVRCPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_JVRCPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_JVRCPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_JVRCPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_JVRCPLUGIN_DLLIMPORT
#   define CNOID_JVRCPLUGIN_DLLEXPORT
#   define CNOID_JVRCPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_JVRCPLUGIN_STATIC
#  define CNOID_JVRCPLUGIN_DLLAPI
#  define CNOID_JVRCPLUGIN_LOCAL
# else
#  ifdef CnoidJVRCPlugin_EXPORTS
#   define CNOID_JVRCPLUGIN_DLLAPI CNOID_JVRCPLUGIN_DLLEXPORT
#  else
#   define CNOID_JVRCPLUGIN_DLLAPI CNOID_JVRCPLUGIN_DLLIMPORT
#  endif
#  define CNOID_JVRCPLUGIN_LOCAL CNOID_JVRCPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_JVRCPLUGIN_DLLAPI
