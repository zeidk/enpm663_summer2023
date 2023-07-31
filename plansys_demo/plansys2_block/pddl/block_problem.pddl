(define (problem blockproblem)
    (:domain blocksimple)

    ;; Define objects
    (:objects
        a b c - block
        gripper1 - gripper
    )

    ;; Define the initial state
    (:init
        (on-table b)
        (on-table a)
        (on c a)
        (clear b)
        (clear c)
        (gripper-empty gripper1)
    )

    ;; Define the goal state
    (:goal
        (and (clear a) (on a b) (on b c)
            (on-table c) (gripper-empty gripper1))
    )
)