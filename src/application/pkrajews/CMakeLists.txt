add_executable(mp_tut
	doNaukiIZabawy/mp_tut.cc
)

target_link_libraries(mp_tut mp discode_sensor)

install(TARGETS mp_tut DESTINATION bin)
