package com.spartronics4915.frc2026.subsystems.vision.filters;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

/**
 * Applies a series of {@link FilterInterface} filters to a list of vision results,
 * returning only those that pass every filter in the pipeline.
 *
 * <p>Filters are applied in the order they were provided to the constructor.
 * For best performance, place cheapest filters first so expensive ones only
 * run on results that have already passed the early checks.
 *
 * <p>Uses a plain loop instead of a stream pipeline to avoid iterator and
 * collector allocation on the hot periodic path.
 */
public class PipelineFilter {

    private final List<FilterInterface> filters;

    public PipelineFilter(List<FilterInterface> filters) {
        this.filters = List.copyOf(filters);
    }

    /**
     * Filters the provided results, returning a new list containing only
     * those that pass every filter in the pipeline.
     *
     * @param results the results to filter; must not be null
     * @return a new list containing only the passing results
     */
    public List<ResultInterface> filter(List<ResultInterface> results) {
        List<ResultInterface> filtered = new ArrayList<>(results.size());
        for (ResultInterface result : results) {
            if (test(result)) filtered.add(result);
        }
        return filtered;
    }

    /**
     * Returns {@code true} if the given result passes every filter in this pipeline.
     *
     * @param result the result to test
     * @return {@code true} if all filters accept the result
     */
    public boolean test(ResultInterface result) {
        for (FilterInterface filter : filters) {
            if (!filter.test(result)) return false;
        }
        return true;
    }

}