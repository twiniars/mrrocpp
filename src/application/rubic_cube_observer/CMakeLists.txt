add_executable(mp_rubic_cube_observer
	mp_rubic_cube_observer.cc
	ecp_mp_tr_rc_windows.cc
	CubeState.cc
	SingleManipulation.cc
)

target_link_libraries(mp_rubic_cube_observer mp)


install(TARGETS
	mp_rubic_cube_observer
	DESTINATION bin)
