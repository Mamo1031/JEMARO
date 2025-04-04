(define (domain boat)
  (:requirements :typing :negative-preconditions :numeric-fluents)
  (:types position)
  (:predicates 
    (engine-on)
  )
  (:functions 
    (boat-x)  ;; Boat's x-coordinate (1 to 20)
    (boat-y)  ;; Boat's y-coordinate (1 to 10)
  )

  ;; Action to turn the engine on
  (:action turn-engine-on
    :parameters ()
    :precondition (not (engine-on))
    :effect (engine-on)
  )

  ;; Action to turn the engine off
  (:action turn-engine-off
    :parameters ()
    :precondition (engine-on)
    :effect (not (engine-on))
  )

  ;; Move straight: increase x by 1 (ensuring x does not exceed the boundary x < 20)
  (:action move-straight
    :parameters ()
    :precondition (and (engine-on) (< (boat-x) 20))
    :effect (increase (boat-x) 1)
  )

  ;; Move left: increase x by 1 and decrease y by 1 (x < 20, y > 1)
  (:action move-left
    :parameters ()
    :precondition (and (engine-on) (< (boat-x) 20) (> (boat-y) 1))
    :effect (and
      (increase (boat-x) 1)
      (decrease (boat-y) 1))
  )

  ;; Move right: increase x by 1 and increase y by 1 (x < 20, y < 10)
  (:action move-right
    :parameters ()
    :precondition (and (engine-on) (< (boat-x) 20) (< (boat-y) 10))
    :effect (and
      (increase (boat-x) 1)
      (increase (boat-y) 1))
  )
)



(define (problem boat-problem)
  (:domain boat)
  (:init
    ;; Initial position of the boat
    (= (boat-x) 1)
    (= (boat-y) 5)
    ;; Initially, the engine is off
    (not (engine-on))
  )
  ;; Goal: x = 19, y = 9
  (:goal (and
    (= (boat-x) 19)
    (= (boat-y) 9)
  ))
)
