//! Density: точные V2-примитивы, поиск и целая raw-операция.
//!
//! [`density_raw_frontend_v1`] несёт batch без memo и явный owned memo успешных попыток.
//! AngleAtom доказывает ранг и оболочку корня; он не повторяет допуск iv160.
//! V2 [`Work`] — диагностика; наблюдаемый бюджет поиска принадлежит [`farey`].

pub mod angle;
pub mod angle_monotone;
pub mod angle_profile;
pub mod atlas_plan;
pub mod atlas_search;
pub mod decimal_projection;
pub mod density_construct;
pub mod density_construct_v1;
pub mod density_expand;
pub mod density_expand_v1;
pub mod density_expression;
pub mod density_expression_order;
pub mod density_quadratic;
pub mod density_residual;
pub mod density_sign;
pub mod density_shell_v2;
pub mod density_trig_nodes;
pub mod density_trig_v1;
pub mod density_angle_value_v1;
pub mod density_laws_v1;
pub mod density_bind_v1;
pub mod density_result_v1;
pub mod density_raw_types_v1;
pub mod density_raw_laws_v1;
pub mod density_raw_result_v1;
pub mod density_raw_noise_v1;
pub mod density_raw_frontend_v1;
pub mod density_attempt_memo_v1;
pub mod density_vector_nodes;
mod error;
pub mod fan_prepare;
pub mod fan_compile;
pub mod farey;
pub mod geometry;
pub mod interval;
pub mod legacy_atan;
pub mod legacy_binding;
mod legacy_atan_table;
mod legacy_fixed;
pub mod legacy_interval;
pub mod legacy_mpf;
pub mod legacy_trig;
pub mod legacy_trig_basecase;
pub mod legacy_trig_eval;
mod metric;
pub mod order;
pub mod order_set;
pub mod scalar_v2;
mod transaction;
pub mod turn_basis;
pub mod turn_rays;
pub mod window_build;

pub use cftuv_radical::{IBig, Priced, Radical, UBig, Work};
pub use error::{DensityFailure, DensityFailureKind, DensityResult, InvalidInput};
pub use metric::{Metric2, Rational, Vector2};
pub use transaction::DensityTransaction;
