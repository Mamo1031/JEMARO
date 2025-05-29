(define (problem warehouse-1-ext421)
  (:domain warehouse-crates-421)
  (:objects 
    m1 m2 - mover
    c1 c3 - normal_crate
    c2 - fragile_crate  
    fl - full_loader
    sl - side_loader
    A - group
  )
  (:init
    (independent m1 m2)
    (independent m2 m1)
    (empty m1)
    (empty m2)
    (at_pause m1)
    (at_pause m2)
    (at_loading_bay m1)
    (at_loading_bay m2)
    (empty fl)
    (idle fl)
    (empty sl)
    (idle sl)
    (on_shelf c1)
    (on_shelf c2)
    (on_shelf c3)
    (pickable c1)
    (pickable c2)
    (pickable c3)
    (= (weight c1) 70)
    (= (weight c2) 20)
    (= (weight c3) 20)
    (= (distance c1) 10)
    (= (distance c2) 20)
    (= (distance c3) 20)
    (no_active_group)
    (no_group c1)
    (is_of_group A c2)
    (is_of_group A c3)
    (= (group-cost) 0)
    )
  (:goal
    (and (loaded c1)
        (loaded c2)
        (loaded c3)
    ))
  (:metric minimize (+ (total-time) (group-cost)))
  )
