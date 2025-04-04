(define (domain gripper-macro)
  (:requirements :strips :typing :conditional-effects)
  (:types room ball gripper)
  (:predicates 
    (room ?r)
    (ball ?b)
    (gripper ?g)
    (at-robby ?r - room)
    (at ?b - ball ?r - room)
    (free ?g - gripper)
    (carry ?b - ball ?g - gripper)
    (connected ?r1 ?r2)
    (together ?b1 - ball ?b2 - ball)
  )
  
  (:action move
    :parameters (?from ?to - room)
    :precondition (and (at-robby ?from)
                       (connected ?from ?to))
    :effect (and (at-robby ?to)
                 (not (at-robby ?from))))
                 
  (:action pick
    :parameters (?obj - ball ?room - room ?gripper - gripper)
    :precondition (and (ball ?obj) (room ?room) (gripper ?gripper)
                       (at ?obj ?room) (at-robby ?room) (free ?gripper))
    :effect (and (carry ?obj ?gripper)
                 (not (at ?obj ?room))
                 (not (free ?gripper))))
                 
  (:action drop
    :parameters (?obj - ball ?room - room ?gripper - gripper)
    :precondition (and (ball ?obj) (room ?room) (gripper ?gripper)
                       (carry ?obj ?gripper) (at-robby ?room))
    :effect (and (at ?obj ?room)
                 (free ?gripper)
                 (not (carry ?obj ?gripper))
                 ))
                 
  ;; Macro operator: transport-two, picks up two balls, moves, drops them, and asserts that they are together.
  (:action transport-two
    :parameters (?from ?to - room ?g1 ?g2 - gripper ?b1 ?b2 - ball)
    :precondition (and 
                    (at-robby ?from)
                    (connected ?from ?to)
                    (free ?g1) (free ?g2)
                    (at ?b1 ?from) (at ?b2 ?from)
                    (not (= ?b1 ?b2)))
    :effect (and 
             ;; Pick up both balls
             (carry ?b1 ?g1)
             (carry ?b2 ?g2)
             (not (at ?b1 ?from))
             (not (at ?b2 ?from))
             (not (free ?g1))
             (not (free ?g2))
             ;; Move robot to destination
             (at-robby ?to)
             (not (at-robby ?from))
             ;; Drop both balls
             (at ?b1 ?to)
             (at ?b2 ?to)
             (free ?g1)
             (free ?g2)
             ;; Entanglement: assert that these two balls are together
             (together ?b1 ?b2)))
)




(define (problem strips-gripper-macro-entangled)
  (:domain gripper-macro)
  (:objects 
    rooma roomb roomc - room
    ball1 ball2 ball3 ball4 - ball
    left right - gripper
  )
  (:init 
    (room rooma) (room roomb) (room roomc)
    (ball ball1) (ball ball2) (ball ball3) (ball ball4)
    (gripper left) (gripper right)
    ;; Define room connections
    (connected rooma roomb) (connected roomb rooma)
    (connected roomb roomc) (connected roomc roomb)
    ;; Set robot's initial location
    (at-robby rooma)
    ;; All balls start in rooma
    (at ball1 rooma)
    (at ball2 rooma)
    (at ball3 rooma)
    (at ball4 rooma)
    ;; Both grippers are free
    (free left) (free right)
  )
  (:goal (and 
          (at ball1 roomb)
          (at ball2 roomb)
          (at ball3 roomb)
          (at ball4 roomb)
          ;; Entangled goal: require ball1 and ball2 to be together, and ball3 and ball4 to be together.
          (together ball1 ball3)
          (together ball2 ball4)))
)
