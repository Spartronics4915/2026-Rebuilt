package com.spartronics4915.frc2026.subsystems.vision.filters;

import java.util.List;
import java.util.stream.Collectors;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

public class PipelineFilter {
    
    private final List<FilterInterface> filters;

    public PipelineFilter(List<FilterInterface> filters) {
        this.filters = List.copyOf(filters);
    }

    public List<ResultInterface> filter(List<ResultInterface> results) {   
        List<ResultInterface> filtered = results.stream()
            .filter(this::test)
            .collect(Collectors.toList());

        return filtered;
    }

    public boolean test(ResultInterface result) {
        return filters.stream().allMatch(filter -> filter.test(result));
    }

}
