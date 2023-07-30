(define (domain kitting)
    (:requirements :strips :typing :adl :durative-actions :fluents)
    (:types
        part tray tray-id agv bin table quadrant part-type-color
    )

    (:functions
        (competition-started)
        (competition-ended)
        (current-num-parts-in-kit)
        (expected-num-parts-in-kit)
    )

    (:predicates
        (in-bin ?x - part ?y - bin)
        (part-in-tray ?x - part-type-color ?y - tray-id ?z - quadrant)
        (on-agv ?x - tray ?y - agv)
        (on-table ?x - tray ?y - table)
        (part-gripper-mounted)
        (tray-gripper-mounted)
        (tray-on-agv ?x - tray)
        (holding-part ?x - part)
        (holding-tray ?x - tray)
        (gripper-empty)
        (has-part-type-color ?x - part ?y - part-type-color)
        (has-tray-id ?x - tray ?y - tray-id)
    )

    (:durative-action PickupTray
        :parameters (?tray - tray ?tray_id - tray-id ?table - table)
        :duration (= ?duration 15)
        :condition (and
            (at start (has-tray-id ?tray ?tray_id))
            (at start (on-table ?tray ?table))
            (at start (tray-gripper-mounted))
            (at start (gripper-empty))
        )
        :effect (and
            (at start (not (on-table ?tray ?table)))
            (at start (not (gripper-empty)))
            (at end (holding-tray ?tray))
        )
    )

    (:durative-action PutDownTray
        :parameters (?tray - tray ?agv - agv)
        :duration (= ?duration 15)
        :condition (and
            (at start (holding-tray ?tray))
        )
        :effect (and
            (at start (not (holding-tray ?tray)))
            (at end (on-agv ?tray ?agv))
            (at end (tray-on-agv ?tray))
            (at end (gripper-empty))
        )
    )

    (:durative-action ChangeToTrayGripper
        :parameters ()
        :duration (= ?duration 5)
        :condition (and
            (at start (= (competition-started) 1))
            (at start (part-gripper-mounted))
            (at start (gripper-empty))
        )
        :effect (and
            (at start (not (part-gripper-mounted)))
            (at end (tray-gripper-mounted))
        )
    )

    (:durative-action ChangeToPartGripper
        :parameters ()
        :duration (= ?duration 5)
        :condition (and
            (at start (tray-gripper-mounted))
            (at start (gripper-empty))
            
        )
        :effect (and
            (at start (not (tray-gripper-mounted)))
            (at end (part-gripper-mounted))
        )
    )

    (:durative-action PickUpPart
        :parameters (?part - part ?part_type_color - part-type-color ?bin - bin)
        :duration (= ?duration 15)
        :condition (and
            (at start (has-part-type-color ?part ?part_type_color))
            (at start (in-bin ?part ?bin))
            (at start (part-gripper-mounted))
            (at start (gripper-empty))
        )
        :effect (and
            (at start (not (in-bin ?part ?bin)))
            (at start (not (gripper-empty)))
            (at end (holding-part ?part))
        )
    )

    (:durative-action PutDownPart
        :parameters (?part - part ?part_type_color - part-type-color ?tray - tray ?tray_id - tray-id ?quadrant - quadrant)
        :duration (= ?duration 15)
        :condition (and
            (at start (holding-part ?part))
            (at start (tray-on-agv ?tray))
            (over all (has-part-type-color ?part ?part_type_color))
            (at start (has-tray-id ?tray ?tray_id))
        )
        :effect (and
            (at start (not (holding-part ?part)))
            (at end (part-in-tray ?part_type_color ?tray_id ?quadrant))
            (at end (gripper-empty))
            (at end (increase (current-num-parts-in-kit) 1))
        )
    )

    (:durative-action StartCompetition
        :parameters ()
        :duration (= ?duration 2)
        :condition (and
            (at start (= (competition-started) 0))
        )
        :effect (and
            (at end (increase (competition-started) 1))
        )
    )

    (:durative-action EndCompetition
        :parameters ()
        :duration (= ?duration 2)
        :condition (and
            (at start (= (competition-ended) 0))
            (over all (= (current-num-parts-in-kit) (expected-num-parts-in-kit)))
        )
        :effect (and
            (at end (increase (competition-ended) 1))
        )
    )

)