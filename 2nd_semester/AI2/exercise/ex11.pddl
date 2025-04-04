(define (domain driverlog)
  (:requirements :strips :typing)
  (:predicates 
     (OBJ ?obj)
     (TRUCK ?truck)
     (LOCATION ?loc)
     (DRIVER ?d)
     (at ?obj ?loc)
     (in ?obj1 ?obj)
     (driving ?d ?v)
     (link ?x ?y)
     (path ?x ?y)
     (empty ?v)
  )

  (:action LOAD-TRUCK
    :parameters (?obj ?truck ?loc)
    :precondition (and (OBJ ?obj) (TRUCK ?truck) (LOCATION ?loc)
                       (at ?truck ?loc) (at ?obj ?loc))
    :effect (and (not (at ?obj ?loc)) (in ?obj ?truck))
  )

  (:action UNLOAD-TRUCK
    :parameters (?obj ?truck ?loc)
    :precondition (and (OBJ ?obj) (TRUCK ?truck) (LOCATION ?loc)
                       (at ?truck ?loc) (in ?obj ?truck))
    :effect (and (not (in ?obj ?truck)) (at ?obj ?loc))
  )

  (:action BOARD-TRUCK
    :parameters (?driver ?truck ?loc)
    :precondition (and (DRIVER ?driver) (TRUCK ?truck) (LOCATION ?loc)
                       (at ?truck ?loc) (at ?driver ?loc) (empty ?truck))
    :effect (and (not (at ?driver ?loc))
                 (driving ?driver ?truck)
                 (not (empty ?truck)))
  )

  (:action DISEMBARK-TRUCK
    :parameters (?driver ?truck ?loc)
    :precondition (and (DRIVER ?driver) (TRUCK ?truck) (LOCATION ?loc)
                       (at ?truck ?loc) (driving ?driver ?truck))
    :effect (and (not (driving ?driver ?truck))
                 (at ?driver ?loc)
                 (empty ?truck))
  )

  (:action DRIVE-TRUCK
    :parameters (?truck ?loc-from ?loc-to ?driver)
    :precondition (and (TRUCK ?truck) (LOCATION ?loc-from) (LOCATION ?loc-to)
                       (DRIVER ?driver) (at ?truck ?loc-from)
                       (driving ?driver ?truck) (link ?loc-from ?loc-to))
    :effect (and (not (at ?truck ?loc-from)) (at ?truck ?loc-to))
  )

  (:action WALK
    :parameters (?driver ?loc-from ?loc-to)
    :precondition (and (DRIVER ?driver) (LOCATION ?loc-from) (LOCATION ?loc-to)
                       (at ?driver ?loc-from) (path ?loc-from ?loc-to))
    :effect (and (not (at ?driver ?loc-from)) (at ?driver ?loc-to))
  )

  ;; Macro operator: transport-truck
  ;; Combines boarding, driving, and disembarking.
  (:action transport-truck
    :parameters (?driver ?truck ?from ?to - LOCATION)
    :precondition (and 
                    (DRIVER ?driver)
                    (TRUCK ?truck)
                    (LOCATION ?from) (LOCATION ?to)
                    (at ?driver ?from)
                    (at ?truck ?from)
                    (empty ?truck)
                    (link ?from ?to))
    :effect (and 
             ;; Simulate boarding
             (not (at ?driver ?from))
             (driving ?driver ?truck)
             (not (empty ?truck))
             ;; Simulate driving: move truck to ?to
             (not (at ?truck ?from))
             (at ?truck ?to)
             ;; Simulate disembarking
             (not (driving ?driver ?truck))
             (at ?driver ?to)
             (empty ?truck))
  )
)



(define (problem DLOG-2-2-2)
  (:domain driverlog)
  (:objects
    driver1 driver2 - DRIVER
    truck1 truck2 - TRUCK
    package1 package2 - OBJ
    s0 s1 s2 p1-0 p1-2 - LOCATION
  )
  (:init
    (at driver1 s0)
    (DRIVER driver1)
    (at driver2 s2)
    (DRIVER driver2)
    (at truck1 s0)
    (empty truck1)
    (TRUCK truck1)
    (at truck2 s0)
    (empty truck2)
    (TRUCK truck2)
    (at package1 s0)
    (OBJ package1)
    (at package2 s0)
    (OBJ package2)
    (LOCATION s0)
    (LOCATION s1)
    (LOCATION s2)
    (LOCATION p1-0)
    (LOCATION p1-2)
    (path s1 p1-0) (path p1-0 s1)
    (path s0 p1-0) (path p1-0 s0)
    (path s1 p1-2) (path p1-2 s1)
    (path s2 p1-2) (path p1-2 s2)
    (link s0 s1) (link s1 s0)
    (link s0 s2) (link s2 s0)
    (link s2 s1) (link s1 s2)
  )
  (:goal (and
    (at driver1 s1)
    (at truck1 s1)
    (at package1 s0)
    (at package2 s0)
  ))
)

