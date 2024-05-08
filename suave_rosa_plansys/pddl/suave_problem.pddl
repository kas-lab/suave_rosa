( define ( problem suave_rosa )
( :domain suave )
( :objects
	bluerov - robot
  p1 - pipeline
  search_pipeline inspect_pipeline - action
)
( :init
  (search_pipeline_action search_pipeline)
  (inspect_pipeline_action inspect_pipeline)

  (action_feasible search_pipeline)
  (action_feasible inspect_pipeline)

  (battery_charged bluerov)
)
( :goal
  ( and
    (pipeline_found p1)
    (pipeline_inspected p1)
  )
)
)
