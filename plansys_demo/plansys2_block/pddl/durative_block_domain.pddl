(define (domain blockdurative)
    (:requirements :strips :typing :adl :durative-actions)
    (:types
        block gripper
    )

    (:predicates
        (on-table ?x - block) ;Block 'x' is on the table
        (on ?x - block ?y - block) ; Block 'x' is on top of block 'y'
        (clear ?x - block) ; Block 'x' is clear (nothing on top)
        (holding ?g - gripper ?x - block) ; Gripper 'g' is holding block 'x'
        (gripper-empty ?g - gripper) ; Gripper 'g' is empty
    )

    ;; pick up a block from the table
    (:durative-action pickup
        :parameters (?b - block ?g - gripper)
        :duration (= ?duration 2)
        :condition (and
            (at start (clear ?b))
            (at start (on-table ?b))
            (at start (gripper-empty ?g))
        )
        :effect (and
            (at start (not (clear ?b)))
            (at start (not (gripper-empty ?g)))
            (at start (not (on-table ?b)))
            (at end (holding ?g ?b))
        )
    )

    ;; put down a block on the table
    (:durative-action putdown
        :parameters (?b - block ?g - gripper)
        :duration (= ?duration 2)
        :condition (at start (holding ?g ?b))
        :effect (and
            (at start(not (holding ?g ?b)))
            (at end(clear ?b))
            (at end(gripper-empty ?g))
            (at end(on-table ?b))
        )
    )

    ;; stack a block on top of another block 
    (:durative-action stack
        :parameters (?b - block ?x - block ?g - gripper)
        :duration (= ?duration 2)
        :condition (and
            (at start(holding ?g ?b))
            (at start (clear ?x))
        )
        :effect (and
            (at start (not (clear ?x)))
            (at start (not (holding ?g ?b)))
            (at end (clear ?b))
            (at end (gripper-empty ?g))
            (at end (on ?b ?x))
        )
    )

    ;; unstack a block from another block 
    (:durative-action unstack
        :parameters (?b - block ?x - block ?g - gripper)
        :duration (= ?duration 2)
        :condition (and
            (at start(on ?b ?x))
            (at start(clear ?b))
            (at start(gripper-empty ?g))
        )
        :effect (and
            (at start (not(gripper-empty ?g)))
            (at start (not (on ?b ?x)))
            (at start (not (clear ?b)))
            (at end(clear ?x))
            (at end (holding ?g ?b))
        )
    )
)