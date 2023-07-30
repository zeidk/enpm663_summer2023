
(define (problem kitting)
    (:domain kitting)
    (:objects
        part1 part2 part3 - part
        bin1 - bin
        QUADRANT1 QUADRANT2 QUADRANT3 QUADRANT4 - quadrant
        tray1 tray2 tray3 - tray
        table1 table2 - table
        BLUEBATTERY - part-type-color
        TRAYID1 TRAYID2 - tray-id
        agv1 agv2 - agv

    )

    (:init
        (non-competition-started)
        (non-competition-ended)
        (non-submitted-order)
        (= (current-num-parts-in-kit) 0)
        (= (expected-num-parts-in-kit) 2)
        (agv-at-kitting-station agv1)
        (agv-at-kitting-station agv2)

        (has-part-type-color part1 BLUEBATTERY)
        (has-part-type-color part2 BLUEBATTERY)
        (has-part-type-color part3 BLUEBATTERY)
        (has-tray-id tray1 TRAYID1)
        (has-tray-id tray2 TRAYID1)
        (has-tray-id tray3 TRAYID2)
        (in-bin part1 bin1)
        (in-bin part2 bin1)
        (in-bin part3 bin1)
        (on-table tray1 table1)
        (on-table tray2 table1)
        (on-table tray3 table2)
        (part-gripper-mounted)
        (gripper-empty)
    )
    (:goal
        (and
            (on-agv tray1 agv1)
            (part-in-tray BLUEBATTERY TRAYID1 QUADRANT1)
            (part-in-tray BLUEBATTERY TRAYID1 QUADRANT2)
            (submitted-order)
        )
    )
)