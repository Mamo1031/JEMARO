(define (domain gripper)
  (:requirements :adl :typing :conditional-effects)
  (:types room ball gripper)
  (:predicates 
    (at-robby ?r - room)
    (at ?b - ball ?r - room)
    (free ?g - gripper)
    (carry ?b - ball ?g - gripper)
  )

  (:action move
    :parameters (?from - room ?to - room)
    :precondition (at-robby ?from)
    :effect (and (at-robby ?to)
                 (not (at-robby ?from)))
  )

  (:action pick
    :parameters (?room - room ?g - gripper)
    :precondition (and (at-robby ?room) (free ?g))
    :effect (forall (?b - ball)
              (when (at ?b ?room)
                    (and (carry ?b ?g)
                         (not (at ?b ?room)))))
  )

  (:action drop
    :parameters (?room - room ?g - gripper)
    :precondition (at-robby ?room)
    :effect (forall (?b - ball)
              (when (carry ?b ?g)
                    (and (at ?b ?room)
                         (not (carry ?b ?g)))))
  )
)



(define (problem gripper-x-1)
  (:domain gripper)
  (:objects 
    rooma roomb - room
    ball1 ball2 ball3 - ball
    left - gripper
  )
  (:init 
    (at-robby rooma)
    (free left)
    (at ball1 rooma)
    (at ball2 rooma)
    (at ball3 rooma)
  )
  (:goal 
    (and (at ball1 roomb)
         (at ball2 roomb)
         (at ball3 roomb))
  )
)