package com.spartronics4915.frc2026.subsystems.vision.filters;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

@FunctionalInterface
public interface ResultFilterInterface {

    boolean test(ResultInterface result);

    default ResultFilterInterface and (ResultFilterInterface other) {
        return (result) -> test(result) && other.test(result);
    }

    default ResultFilterInterface or (ResultFilterInterface other) {
        return (result) -> test(result) || other.test(result);
    }

    default ResultFilterInterface negate() {
        return (result) -> !test(result);
    }
}
