(define (problem dispose-4-1)
(:domain dispose)

 (:objects     o1

    p1-1
    p1-2
    p1-3
    p1-4
    p2-1
    p2-2
    p2-3
    p2-4
    p3-1
    p3-2
    p3-3
    p3-4
    p4-1
    p4-2
    p4-3
    p4-4

    )
   (:init
     (at p2-2)
     (trash-at p1-1)     (adj p1-1 p2-1)
     (adj p2-1 p1-1)

     (adj p1-2 p2-2)
     (adj p2-2 p1-2)

     (adj p1-3 p2-3)
     (adj p2-3 p1-3)

     (adj p1-4 p2-4)
     (adj p2-4 p1-4)

     (adj p2-1 p3-1)
     (adj p3-1 p2-1)

     (adj p2-2 p3-2)
     (adj p3-2 p2-2)

     (adj p2-3 p3-3)
     (adj p3-3 p2-3)

     (adj p2-4 p3-4)
     (adj p3-4 p2-4)

     (adj p3-1 p4-1)
     (adj p4-1 p3-1)

     (adj p3-2 p4-2)
     (adj p4-2 p3-2)

     (adj p3-3 p4-3)
     (adj p4-3 p3-3)

     (adj p3-4 p4-4)
     (adj p4-4 p3-4)


     (adj p1-1 p1-2)
     (adj p1-2 p1-1)

     (adj p2-1 p2-2)
     (adj p2-2 p2-1)

     (adj p3-1 p3-2)
     (adj p3-2 p3-1)

     (adj p4-1 p4-2)
     (adj p4-2 p4-1)

     (adj p1-2 p1-3)
     (adj p1-3 p1-2)

     (adj p2-2 p2-3)
     (adj p2-3 p2-2)

     (adj p3-2 p3-3)
     (adj p3-3 p3-2)

     (adj p4-2 p4-3)
     (adj p4-3 p4-2)

     (adj p1-3 p1-4)
     (adj p1-4 p1-3)

     (adj p2-3 p2-4)
     (adj p2-4 p2-3)

     (adj p3-3 p3-4)
     (adj p3-4 p3-3)

     (adj p4-3 p4-4)
     (adj p4-4 p4-3)


     (oneof
        (obj-at o1 p1-1)
        (obj-at o1 p1-2)
        (obj-at o1 p1-3)
        (obj-at o1 p1-4)
        (obj-at o1 p2-1)
        (obj-at o1 p2-2)
        (obj-at o1 p2-3)
        (obj-at o1 p2-4)
        (obj-at o1 p3-1)
        (obj-at o1 p3-2)
        (obj-at o1 p3-3)
        (obj-at o1 p3-4)
        (obj-at o1 p4-1)
        (obj-at o1 p4-2)
        (obj-at o1 p4-3)
        (obj-at o1 p4-4)
     )

        (unknown (obj-at o1 p1-1))
        (unknown (obj-at o1 p1-2))
        (unknown (obj-at o1 p1-3))
        (unknown (obj-at o1 p1-4))
        (unknown (obj-at o1 p2-1))
        (unknown (obj-at o1 p2-2))
        (unknown (obj-at o1 p2-3))
        (unknown (obj-at o1 p2-4))
        (unknown (obj-at o1 p3-1))
        (unknown (obj-at o1 p3-2))
        (unknown (obj-at o1 p3-3))
        (unknown (obj-at o1 p3-4))
        (unknown (obj-at o1 p4-1))
        (unknown (obj-at o1 p4-2))
        (unknown (obj-at o1 p4-3))
        (unknown (obj-at o1 p4-4))

    )
    (:goal (disposed o1)))
