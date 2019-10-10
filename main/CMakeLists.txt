SubDirList(SUBDIRS .)

add_subdirectory(FactorGraphEstimatorTester)

FOREACH(subdir ${SUBDIRS})
  ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()

set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)