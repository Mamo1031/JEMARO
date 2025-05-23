(define (domain grid)
  (:requirements :strips :typing)
  (:predicates 
    (conn ?x ?y)
    (key-shape ?k ?s)
    (lock-shape ?x ?s)
    (at ?r ?x)
    (at-robot ?x)
    (place ?p)
    (key ?k)
    (shape ?s)
    (locked ?x)
    (holding ?k)
    (open ?x)
    (arm-empty)
  )

  (:action unlock
    :parameters (?curpos ?lockpos ?key ?shape)
    :precondition (and (place ?curpos) (place ?lockpos) (key ?key) (shape ?shape)
                       (conn ?curpos ?lockpos) (key-shape ?key ?shape)
                       (lock-shape ?lockpos ?shape) (at-robot ?curpos) 
                       (locked ?lockpos) (holding ?key))
    :effect (and (open ?lockpos) (not (locked ?lockpos)))
  )

  (:action move
    :parameters (?curpos ?nextpos)
    :precondition (and (place ?curpos) (place ?nextpos)
                       (at-robot ?curpos) (conn ?curpos ?nextpos) (open ?nextpos))
    :effect (and (at-robot ?nextpos) (not (at-robot ?curpos)))
  )

  (:action pickup
    :parameters (?curpos ?key)
    :precondition (and (place ?curpos) (key ?key) 
                       (at-robot ?curpos) (at ?key ?curpos) (arm-empty))
    :effect (and (holding ?key)
                 (not (at ?key ?curpos)) (not (arm-empty)))
  )

  (:action putdown
    :parameters (?curpos ?key)
    :precondition (and (place ?curpos) (key ?key) 
                       (at-robot ?curpos) (holding ?key))
    :effect (and (arm-empty) (at ?key ?curpos) (not (holding ?key)))
  )
)



(define (problem strips-grid-y-1)
   (:domain grid)
   (:objects node0-0 node0-1 node0-2 node0-3 node0-4 node1-0
             node1-1 node1-2 node1-3 node1-4 node2-0 node2-1 node2-2
             node2-3 node2-4 node3-0 node3-1 node3-2 node3-3 node3-4
             node4-0 node4-1 node4-2 node4-3 node4-4 triangle diamond
             square circle key0 key1 key2 key3 key4 key5 key6 key7 key8)
   (:init (arm-empty)
          (place node0-0)
          (place node0-1)
          (place node0-2)
          (place node0-3)
          (place node0-4)
          (place node1-0)
          (place node1-1)
          (place node1-2)
          (place node1-3)
          (place node1-4)
          (place node2-0)
          (place node2-1)
          (place node2-2)
          (place node2-3)
          (place node2-4)
          (place node3-0)
          (place node3-1)
          (place node3-2)
          (place node3-3)
          (place node3-4)
          (place node4-0)
          (place node4-1)
          (place node4-2)
          (place node4-3)
          (place node4-4)
          (shape triangle)
          (shape diamond)
          (shape square)
          (shape circle)
          (conn node0-0 node1-0)
          (conn node0-0 node0-1)
          (conn node0-1 node1-1)
          (conn node0-1 node0-2)
          (conn node0-1 node0-0)
          (conn node0-2 node1-2)
          (conn node0-2 node0-3)
          (conn node0-2 node0-1)
          (conn node0-3 node1-3)
          (conn node0-3 node0-4)
          (conn node0-3 node0-2)
          (conn node0-4 node1-4)
          (conn node0-4 node0-3)
          (conn node1-0 node2-0)
          (conn node1-0 node0-0)
          (conn node1-0 node1-1)
          (conn node1-1 node2-1)
          (conn node1-1 node0-1)
          (conn node1-1 node1-2)
          (conn node1-1 node1-0)
          (conn node1-2 node2-2)
          (conn node1-2 node0-2)
          (conn node1-2 node1-3)
          (conn node1-2 node1-1)
          (conn node1-3 node2-3)
          (conn node1-3 node0-3)
          (conn node1-3 node1-4)
          (conn node1-3 node1-2)
          (conn node1-4 node2-4)
          (conn node1-4 node0-4)
          (conn node1-4 node1-3)
          (conn node2-0 node3-0)
          (conn node2-0 node1-0)
          (conn node2-0 node2-1)
          (conn node2-1 node3-1)
          (conn node2-1 node1-1)
          (conn node2-1 node2-2)
          (conn node2-1 node2-0)
          (conn node2-2 node3-2)
          (conn node2-2 node1-2)
          (conn node2-2 node2-3)
          (conn node2-2 node2-1)
          (conn node2-3 node3-3)
          (conn node2-3 node1-3)
          (conn node2-3 node2-4)
          (conn node2-3 node2-2)
          (conn node2-4 node3-4)
          (conn node2-4 node1-4)
          (conn node2-4 node2-3)
          (conn node3-0 node4-0)
          (conn node3-0 node2-0)
          (conn node3-0 node3-1)
          (conn node3-1 node4-1)
          (conn node3-1 node2-1)
          (conn node3-1 node3-2)
          (conn node3-1 node3-0)
          (conn node3-2 node4-2)
          (conn node3-2 node2-2)
          (conn node3-2 node3-3)
          (conn node3-2 node3-1)
          (conn node3-3 node4-3)
          (conn node3-3 node2-3)
          (conn node3-3 node3-4)
          (conn node3-3 node3-2)
          (conn node3-4 node4-4)
          (conn node3-4 node2-4)
          (conn node3-4 node3-3)
          (conn node4-0 node3-0)
          (conn node4-0 node4-1)
          (conn node4-1 node3-1)
          (conn node4-1 node4-2)
          (conn node4-1 node4-0)
          (conn node4-2 node3-2)
          (conn node4-2 node4-3)
          (conn node4-2 node4-1)
          (conn node4-3 node3-3)
          (conn node4-3 node4-4)
          (conn node4-3 node4-2)
          (conn node4-4 node3-4)
          (conn node4-4 node4-3)
          (locked node4-3)
          (lock-shape node4-3 square)
          (locked node4-4)
          (lock-shape node4-4 square)
          (locked node3-4)
          (lock-shape node3-4 square)
          (locked node3-3)
          (lock-shape node3-3 square)
          (locked node2-3)
          (lock-shape node2-3 square)
          (locked node2-2)
          (lock-shape node2-2 square)
          (locked node3-2)
          (lock-shape node3-2 square)
          (locked node4-2)
          (lock-shape node4-2 square)
          (open node0-0)
          (open node0-1)
          (open node0-2)
          (open node0-3)
          (open node0-4)
          (open node1-0)
          (open node1-1)
          (open node1-2)
          (open node1-3)
          (open node1-4)
          (open node2-0)
          (open node2-1)
          (open node2-4)
          (open node3-0)
          (open node3-1)
          (open node4-0)
          (open node4-1)
          (key key0)
          (key-shape key0 triangle)
          (at key0 node2-3)
          (key key1)
          (key-shape key1 circle)
          (at key1 node1-3)
          (key key2)
          (key-shape key2 diamond)
          (at key2 node0-4)
          (key key3)
          (key-shape key3 square)
          (at key3 node0-2)
          (key key4)
          (key-shape key4 square)
          (at key4 node3-3)
          (key key5)
          (key-shape key5 triangle)
          (at key5 node4-1)
          (key key6)
          (key-shape key6 triangle)
          (at key6 node4-4)
          (key key7)
          (key-shape key7 circle)
          (at key7 node3-4)
          (key key8)
          (key-shape key8 triangle)
          (at key8 node2-2)
          (at-robot node2-4))
   (:goal (and (at key0 node1-1))))