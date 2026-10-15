//! Неприменённый чистый draft. Ни dispatch, ни сертификатов, ни внешних эффектов.
pub mod model;
mod owner;
mod relations;
pub use model::*;
pub use relations::evaluate_patch;

mod decimal_text;
pub use decimal_text::{decimal_text_to_rat, DecimalTextDecline, MAX_DECIMAL_TEXT_BYTES, MAX_DECIMAL_EXPONENT_ABS, MAX_DECIMAL_POWER_OF_TEN};
