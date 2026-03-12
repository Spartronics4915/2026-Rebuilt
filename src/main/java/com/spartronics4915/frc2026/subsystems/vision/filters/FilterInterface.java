package com.spartronics4915.frc2026.subsystems.vision.filters;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

public interface FilterInterface {

    boolean test(ResultInterface result);

    default FilterInterface and (FilterInterface other) {
        return (result) -> test(result) && other.test(result);
    }

    default FilterInterface or (FilterInterface other) {
        return (result) -> test(result) || other.test(result);
    }

    default FilterInterface negate() {
        return (result) -> !test(result);
    }
    
}