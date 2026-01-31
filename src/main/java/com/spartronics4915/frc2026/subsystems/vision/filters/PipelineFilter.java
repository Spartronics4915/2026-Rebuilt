package com.spartronics4915.frc2026.subsystems.vision.filters;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult;

public class PipelineFilter {
    private final List<ResultFilterInterface> filters;

    private PipelineFilter(List<ResultFilterInterface> filters) {
        this.filters = List.copyOf(filters);
    }

    public List<CameraResult> filter(List<CameraResult> results) {
        List<CameraResult> filtered = new ArrayList<>(results.size());
        
        for (CameraResult result : results) {
            if (test(result)) filtered.add(result);
        }
        
        return filtered;
    }

    public boolean test(CameraResult result) {
        for (ResultFilterInterface filter : filters) {
            if (!filter.test(result)) return false;
        }
        return true;
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
