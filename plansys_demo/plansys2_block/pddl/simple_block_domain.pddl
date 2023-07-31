(define (domain blocksimple)
    (:requirements :strips :typing :adl)
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
    (:action pickup
        :parameters (?b - block ?g - gripper)
        :precondition (and (clear ?b) (on-table ?b) (gripper-empty ?g))
        :effect (and (not (clear ?b)) (not (gripper-empty ?g)) (not (on-table ?b)) (holding ?g ?b))
    )

    ;; put down a block on the table
    (:action putdown
        :parameters (?b - block ?g - gripper)
        :precondition (holding ?g ?b)
        :effect (and (clear ?b) (gripper-empty ?g) (on-table ?b) (not (holding ?g ?b)))
    )

    ;; stack a block on top of another block
    (:action stack
        :parameters (?b - block ?x - block ?g - gripper)
        :precondition (and (holding ?g ?b) (clear ?x))
        :effect (and (clear ?b) (gripper-empty ?g) (not (holding ?g ?b)) (on ?b ?x) (not (clear ?x)))
    )

    ;; unstack a block from another block
    (:action unstack
        :parameters (?b - block ?x - block ?g - gripper)
        :precondition (and (on ?b ?x) (clear ?b) (gripper-empty ?g))
        :effect (and (clear ?x) (not(gripper-empty ?g)) (not (on ?b ?x)) (holding ?g ?b) (not (clear ?b)))
    )

)