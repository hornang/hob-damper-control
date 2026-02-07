use core::time::Duration;
use discrete_pid::{pid, time};

fn pid() -> Vec<f32> {
    let loop_time = Duration::from_millis(100);
    let cfg = pid::PidConfigBuilder::default()
        .kp(1.0)
        .ki(5.0)
        .sample_time(loop_time)
        .filter_tc(0.1)
        .build()
        .expect("Failed to build a PID configuration");

    let mut controller = pid::FuncPidController::new(cfg);
    let mut ctx = pid::PidContext::new_uninit();

    let setpoint = 2.0;
    let mut outputs = Vec::new();
    let mut timestamp = time::SecondsF64(0.0);

    for i in 0..10 {
        let measurement = 1.0; // Fixed measurement for testing
        println!("Iteration {} at {:.1}s", i, timestamp.0);

        let (output, new_ctx) = controller.compute(ctx, measurement, setpoint, timestamp, None);
        ctx = new_ctx;
        outputs.push(output);

        timestamp.0 += 0.1;
    }

    outputs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid() {
        let outputs = pid();

        let expected = vec![1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0];

        assert_eq!(
            outputs.len(),
            expected.len(),
            "Output vector length mismatch"
        );

        for (i, (actual, expected_val)) in outputs.iter().zip(expected.iter()).enumerate() {
            let error = (actual - expected_val).abs();
            assert!(
                error < 0.01,
                "Iteration {}: expected ~{:.4}, got {:.4} (error: {:.4})",
                i,
                expected_val,
                actual,
                error
            );
            println!(
                "Iteration {}: {:.6} (expected ~{:.4})",
                i, actual, expected_val
            );
        }
    }
}
