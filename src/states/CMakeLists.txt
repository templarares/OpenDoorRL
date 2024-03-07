add_fsm_state_simple(OpenDoorRL_Initial)
add_fsm_state_simple(Teleport)
add_fsm_state_simple(Rewind)
add_fsm_state_simple(RLInterface)
add_fsm_state_simple(RLMeta)
add_fsm_state_simple(RLParallel)
target_link_libraries(RLMeta PRIVATE mc_rtc::fsm::states::Meta)
target_link_libraries(RLParallel PRIVATE mc_rtc::fsm::states::Parallel)
target_link_libraries(RLInterface PRIVATE mc_rtc::fsm::states::MetaTasks)


add_fsm_data_directory(data)
