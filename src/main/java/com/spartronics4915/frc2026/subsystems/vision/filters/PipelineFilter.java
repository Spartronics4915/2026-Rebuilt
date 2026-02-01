package com.spartronics4915.frc2026.subsystems.vision.filters;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

public class PipelineFilter {
    private final List<ResultFilterInterface> filters;

    private PipelineFilter(List<ResultFilterInterface> filters) {
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

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private final List<ResultFilterInterface> filters = new ArrayList<>();

        public Builder addFilter(ResultFilterInterface filter) {
            filters.add(filter);
            return this;
        }

        public Builder addFilters(ResultFilterInterface... filters) {
            for (ResultFilterInterface filter : filters) this.filters.add(filter);
            return this;
        }

        public PipelineFilter build() {
            return new PipelineFilter(filters);
        }
    }
}
