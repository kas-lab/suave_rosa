(define (domain suave)
    (:requirements :strips :typing :adl :fluents :durative-actions :negative-preconditions)

    (:types
        pipeline
        robot
        action
    )

    (:predicates
        (pipeline_found ?p - pipeline)
        (pipeline_inspected ?p - pipeline)

        (search_pipeline_action ?a - action)
        (inspect_pipeline_action ?a - action)
        (recharge_action ?a - action)

        (action_feasible ?a - action)

        (robot_started ?r - robot)
    )

    (:functions
    )

    (:durative-action start_robot
      :parameters (?r - robot)
      :duration ( = ?duration 5)
      :condition (and
      )
      :effect (and
        (at end(robot_started ?r))
      )
    )

    (:durative-action search_pipeline
      :parameters (?a - action ?p - pipeline ?r - robot)
      :duration ( = ?duration 5)
      :condition (and
        (over all (robot_started ?r))
        (over all (search_pipeline_action ?a))
        (over all (action_feasible ?a))
      )
      :effect (and
        (at end(pipeline_found ?p))
      )
    )

    (:durative-action inspect_pipeline
      :parameters (?a - action ?p - pipeline ?r - robot)
      :duration ( = ?duration 5)
      :condition (and
        (over all (robot_started ?r))
        (over all (pipeline_found ?p))
        (over all (inspect_pipeline_action ?a))
        (over all (action_feasible ?a))
      )
      :effect (and
        (at end(pipeline_inspected ?p))
      )
    )

    (:durative-action recharge
      :parameters (?a ?a1 ?a2 - action ?r - robot)
      :duration ( = ?duration 5)
      :condition (and
        (over all (robot_started ?r))
        (over all (recharge_action ?a))
        (over all (action_feasible ?a))
        (over all (search_pipeline_action ?a1))
        (over all (inspect_pipeline_action ?a2))
      )
      :effect (and
        (at end(action_feasible ?a1))
        (at end(action_feasible ?a2))
      )
    )
)
