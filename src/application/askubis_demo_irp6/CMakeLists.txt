add_executable(mp_askubis_demo_irp6
	mp_t_askubis_demo_irp6.cc
)

target_link_libraries(mp_askubis_demo_irp6 mp)

install(TARGETS mp_askubis_demo_irp6 DESTINATION bin)
