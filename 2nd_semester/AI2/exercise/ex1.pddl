(define (domain gripper)
  (:requirements :adl :typing)
  (:types room ball gripper)
  (:predicates 
    (at-robby ?r1 - room)
    (at ?b - ball ?r - room)
    (connected ?r1 - room ?r2 - room)
    (free1 ?g - gripper)
    (free2 ?g - gripper)
    (carry1 ?o - ball ?g - gripper)
    (carry2 ?o - ball ?g - gripper)
  )

  (:action move
    :parameters (?from - room ?to - room)
    :precondition (and (at-robby ?from)
                       (connected ?from ?to))
    :effect (and (at-robby ?to)
                 (not (at-robby ?from)))
  )

  (:action pick
    :parameters (?obj - ball ?room - room ?gripper - gripper)
    :precondition (and (at ?obj ?room)
                       (at-robby ?room)
                       (or (free1 ?gripper) (free2 ?gripper)))
    :effect (and (not (at ?obj ?room))
                 (when (free1 ?gripper)
                       (and (carry1 ?obj ?gripper)
                            (not (free1 ?gripper))))
                 (when (and (not (free1 ?gripper)) (free2 ?gripper))
                       (and (carry2 ?obj ?gripper)
                              (not (free2 ?gripper))))
    )
  )

  (:action drop
    :parameters (?obj - ball ?room - room ?gripper - gripper)
    :precondition (and (at-robby ?room)
                       (or (carry1 ?obj ?gripper) (carry2 ?obj ?gripper)))
    :effect (and (at ?obj ?room)
                 (when (carry1 ?obj ?gripper)
                       (and (free1 ?gripper)
                            (not (carry1 ?obj ?gripper))))
                 (when (carry2 ?obj ?gripper)
                       (and (free2 ?gripper)
                            (not (carry2 ?obj ?gripper))))
    )
  )
)



(define (problem gripper-x-extended)
  (:domain gripper)
  (:objects 
    rooma roomb roomc - room
    ball1 ball2 ball3 - ball
    left right - gripper
  )
  (:init 
    (at-robby rooma)
    (free1 left) (free2 left)
    (free1 right) (free2 right)
    (at ball1 rooma)
    (at ball2 rooma)
    (at ball3 rooma)
    (connected rooma roomb)
    (connected roomb rooma)
  )
  (:goal 
    (and (at ball2 roomb)
         (at ball1 roomb))
  )
)
