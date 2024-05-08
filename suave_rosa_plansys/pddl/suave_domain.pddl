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
        (battery_charged ?r - robot)

        (search_pipeline_action ?a - action)
        (inspect_pipeline_action ?a - action)

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
            (over all (battery_charged ?r))
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
            (over all (battery_charged ?r))
            (over all (inspect_pipeline_action ?a))
            (over all (action_feasible ?a))
        )
        :effect (and
            (at end(pipeline_inspected ?p))
        )
    )
)
