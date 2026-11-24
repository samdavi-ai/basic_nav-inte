use crate::ast::KValue;
use crate::planner::KPlan;
use crate::planner::PlannedStage;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RuntimeError {
    #[error("Stage execution failed: {0}")]
    StageFailed(String),
    #[error("Timeout during stage {0}::{1}")]
    Timeout(String, String),
    #[error("Dispatcher error: {0}")]
    DispatcherError(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StageResult {
    pub stage_index: usize,
    pub noun: String,
    pub verb: String,
    pub success: bool,
    pub output: Option<KValue>,
    pub execution_time_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionSummary {
    pub total_stages: usize,
    pub completed_stages: usize,
    pub success: bool,
    pub stage_results: Vec<StageResult>,
}

pub struct ExecutionContext {
    pub variables: HashMap<String, KValue>,
    pub pipe_context: Option<KValue>,
}

impl ExecutionContext {
    pub fn new() -> Self {
        Self {
            variables: HashMap::new(),
            pipe_context: None,
        }
    }
}

impl Default for ExecutionContext {
    fn default() -> Self {
        Self::new()
    }
}

pub struct KRuntime;

impl KRuntime {
    pub fn new() -> Self {
        Self
    }

    /// Executes a planned pipeline deterministically
    pub fn execute(&self, plan: &KPlan) -> Result<ExecutionSummary, RuntimeError> {
        self.execute_with(plan, |stage, _| {
            Err(RuntimeError::DispatcherError(format!(
                "No executor bound for {}::{}",
                stage.stage.noun, stage.stage.verb
            )))
        })
    }

    pub fn execute_with<F>(
        &self,
        plan: &KPlan,
        mut dispatch: F,
    ) -> Result<ExecutionSummary, RuntimeError>
    where
        F: FnMut(&PlannedStage, Option<&KValue>) -> Result<Option<KValue>, RuntimeError>,
    {
        let mut ctx = ExecutionContext::new();
        let mut results = Vec::new();

        for (idx, planned_stage) in plan.stages.iter().enumerate() {
            let stage = &planned_stage.stage;
            let start = std::time::Instant::now();

            let stage_output = dispatch(planned_stage, ctx.pipe_context.as_ref())?;

            let duration = start.elapsed().as_millis() as u64;
            ctx.pipe_context = stage_output.clone();

            results.push(StageResult {
                stage_index: idx,
                noun: stage.noun.clone(),
                verb: stage.verb.clone(),
                success: true,
                output: stage_output,
                execution_time_ms: duration,
            });
        }

        Ok(ExecutionSummary {
            total_stages: plan.stages.len(),
            completed_stages: results.len(),
            success: true,
            stage_results: results,
        })
    }
}

impl Default for KRuntime {
    fn default() -> Self {
        Self::new()
    }
}
