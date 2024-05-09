( define ( problem suave_rosa )
( :domain suave )
( :objects
	bluerov - robot
  p1 - pipeline
  search_pipeline inspect_pipeline recharge - action
)
( :init
  (search_pipeline_action search_pipeline)
  (inspect_pipeline_action inspect_pipeline)
  (recharge_action recharge)
)
( :goal
  ( and
    (pipeline_found p1)
    (pipeline_inspected p1)
  )
)
)
