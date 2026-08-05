add_subdirectory(sample_component)

set(APP_DISPATCHER_SOURCE_DIR ${APP_SUPPORT_PACKAGE}/app_dispatcher)
set(LCM_SOURCE_DIR ${APP_SUPPORT_PACKAGE}/life_cycle_manager)
set(APP_BIN ${CMAKE_HOME_DIRECTORY}/bin)
set(APP_LIBS ${CMAKE_HOME_DIRECTORY}/libs)
set(APP_RES ${CMAKE_HOME_DIRECTORY}/res)
set(APP_STORAGE ${CMAKE_HOME_DIRECTORY}/storage)

# Get app_dispatcher binary and manifests from AppSupportPackage
file(MAKE_DIRECTORY ${APP_LIBS})
file(COPY ${APP_DISPATCHER_SOURCE_DIR} DESTINATION ${APP_LIBS})

# Create res, res/models, res, res/settings
file(MAKE_DIRECTORY ${APP_RES})
file(MAKE_DIRECTORY ${APP_RES}/models)
file(MAKE_DIRECTORY ${APP_RES}/settings)

# Create storage, storage/settings
file(MAKE_DIRECTORY ${APP_STORAGE})
file(MAKE_DIRECTORY ${APP_STORAGE}/settings)

# Get application execution binary and LCM manifests from AppSupportPackage
# - this operation is executed in cmake command running time, not build time
file(MAKE_DIRECTORY ${APP_BIN})
file(COPY ${LCM_SOURCE_DIR}/ DESTINATION ${APP_BIN}/)
file(RENAME ${APP_BIN}/life_cycle_manager ${APP_BIN}/${PROJECT_NAME})

# (Optional) Get 3rd Party library from Hanwha Vision Open Platform Docker Image
#set(LIB_3RD_PARTY_DIR ${WISENET_PRODUCT_RELEASE_PATH}/3rd_party/lib)
#file(COPY ${LIB_3RD_PARTY_DIR}/libssl.so DESTINATION ${APP_LIBS})
#file(COPY ${LIB_3RD_PARTY_DIR}/libcrypto.so DESTINATION ${APP_LIBS})

# install PLifeCycleManagermanifest.json
install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/PLifeCycleManagermanifest.json DESTINATION ${APP_BIN})