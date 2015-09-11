add_cnoid_body_customizer(CabinetBoxCustomizer CabinetBoxCustomizer.cpp ${mofile})
target_link_libraries(CabinetBoxCustomizer CnoidUtil)

add_cnoid_body_customizer(HoseCustomizer HoseCustomizer.cpp ${mofile})
target_link_libraries(HoseCustomizer CnoidUtil)

