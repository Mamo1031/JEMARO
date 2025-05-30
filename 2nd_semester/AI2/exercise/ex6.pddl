(define (domain zenotravel)
  (:types locatable city - object
          aircraft person - locatable)
  (:predicates 
    (located ?x - locatable ?c - city)
    (in ?p - person ?a - aircraft)
  )
  (:functions 
    (fuel ?a - aircraft)
    (distance ?c1 - city ?c2 - city)
    (slow-burn ?a - aircraft)
    (fast-burn ?a - aircraft)
    (capacity ?a - aircraft)
    (total-fuel-used)
    (onboard ?a - aircraft)
    (zoom-limit ?a - aircraft)
  )
  
  (:action board
    :parameters (?p - person ?a - aircraft ?c - city)
    :precondition (and (located ?p ?c)
                       (located ?a ?c))
    :effect (and (not (located ?p ?c))
                 (in ?p ?a)
                 (increase (onboard ?a) 1))
  )

  (:action debark
    :parameters (?p - person ?a - aircraft ?c - city)
    :precondition (and (in ?p ?a)
                       (located ?a ?c))
    :effect (and (not (in ?p ?a))
                 (located ?p ?c)
                 (decrease (onboard ?a) 1))
  )

  (:action fly-slow
    :parameters (?a - aircraft ?c1 ?c2 - city)
    :precondition (and (located ?a ?c1)
                       (>= (fuel ?a) (* (distance ?c1 ?c2) (slow-burn ?a))))
    :effect (and (not (located ?a ?c1))
                 (located ?a ?c2)
                 (increase (total-fuel-used) (* (distance ?c1 ?c2) (slow-burn ?a)))
                 (decrease (fuel ?a) (* (distance ?c1 ?c2) (slow-burn ?a))))
  )

  (:action fly-fast
    :parameters (?a - aircraft ?c1 ?c2 - city)
    :precondition (and (located ?a ?c1)
                       (>= (fuel ?a) (* (distance ?c1 ?c2) (fast-burn ?a)))
                       (<= (onboard ?a) (zoom-limit ?a)))
    :effect (and (not (located ?a ?c1))
                 (located ?a ?c2)
                 (increase (total-fuel-used) (* (distance ?c1 ?c2) (fast-burn ?a)))
                 (decrease (fuel ?a) (* (distance ?c1 ?c2) (fast-burn ?a))))
  )

  (:action fly-extrafast
    :parameters (?a - aircraft ?c1 ?c2 - city)
    :precondition (and (located ?a ?c1)
                       (= (fuel ?a) (capacity ?a)))
    :effect (and (not (located ?a ?c1))
                 (located ?a ?c2)
                 (assign (fuel ?a) 0)
                 (increase (total-fuel-used) (capacity ?a)))
  )

  (:action refuel	
    :parameters (?a - aircraft)
    :precondition (and (> (capacity ?a) (fuel ?a)))
    :effect (assign (fuel ?a) (capacity ?a))
  )
)



(define (problem ZTRAVEL-1-2)
  (:domain zenotravel)
  (:objects
    plane1 - aircraft
    person1 - person
    person2 - person
    person3 - person
    city0 - city
    city1 - city
    city2 - city
  )
  (:init
    (located plane1 city0)
    (= (capacity plane1) 6000)
    (= (fuel plane1) 6000)
    (= (slow-burn plane1) 4)
    (= (fast-burn plane1) 15)
    (= (onboard plane1) 0)
    (= (zoom-limit plane1) 8)
    (located person1 city0)
    (located person2 city0)
    (located person3 city1)
    (= (distance city0 city0) 0)
    (= (distance city0 city1) 678)
    (= (distance city0 city2) 775)
    (= (distance city1 city0) 678)
    (= (distance city1 city1) 0)
    (= (distance city1 city2) 810)
    (= (distance city2 city0) 775)
    (= (distance city2 city1) 810)
    (= (distance city2 city2) 0)
    (= (total-fuel-used) 0)
  )
  (:goal (and	
    (located person1 city2)
    (located person2 city1)
    (located person3 city2)
  ))
  (:metric maximize (total-fuel-used))
)
