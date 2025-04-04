(define (domain ferry)
  (:requirements :strips :typing)
  (:types location car ferry)
  (:types port-ferry all-ferry - ferry)
  
  (:predicates 
    (not-eq ?x ?y)
    (car ?c - car)
    (location ?l - location)
    (port ?l - location)
    (at-ferry ?f - ferry ?l - location)
    (at-car ?c - car ?l - location)
    (empty ?f - ferry)
    (on ?c - car)
    (all-ferry ?f - ferry)
  )

  (:action sail
    :parameters (?f - ferry ?from - location ?to - location)
    :precondition (and (not-eq ?from ?to)
                       (location ?from) (location ?to)
                       (at-ferry ?f ?from))
    :effect (and (at-ferry ?f ?to)
                 (not (at-ferry ?f ?from)))
  )

  (:action board
    :parameters (?c - car ?f - ferry ?loc - location)
    :precondition (and (car ?c)
                       (location ?loc)
                       (at-car ?c ?loc)
                       (at-ferry ?f ?loc)
                       (empty ?f)
                       (or (all-ferry ?f) (port ?loc)))
    :effect (and (on ?c)
                 (not (at-car ?c ?loc))
                 (not (empty ?f)))
  )

  (:action debark
    :parameters (?c - car ?f - ferry ?loc - location)
    :precondition (and (car ?c)
                       (location ?loc)
                       (on ?c)
                       (at-ferry ?f ?loc)
                       (or (all-ferry ?f) (port ?loc)))
    :effect (and (at-car ?c ?loc)
                 (empty ?f)
                 (not (on ?c)))
  )
)



(define (problem ferry-problem)
  (:domain ferry)
  (:objects 
    l0 l1 l2 l3 l4 - location
    c0 c1 c2 - car
    f0 - port-ferry
    f1 - all-ferry
  )
  (:init
    (location l0) (location l1) (location l2) (location l3) (location l4)
    (car c0) (car c1) (car c2)
    (port l1) (port l3)
    
    (at-ferry f0 l1)
    (at-ferry f1 l2)
    
    (at-car c0 l1)
    (at-car c1 l2)
    (at-car c2 l4)
    
    (empty f0)
    (empty f1)
    
    (not-eq l0 l1) (not-eq l0 l2) (not-eq l0 l3) (not-eq l0 l4)
    (not-eq l1 l2) (not-eq l1 l3) (not-eq l1 l4)
    (not-eq l2 l3) (not-eq l2 l4)
    (not-eq l3 l4)
  )
  (:goal
    (and
      (at-car c0 l3)
    )
  )
)
