use cftuv_density::density_construct_v1::{ConstructorV1, OwnedSignErrorV1};
use cftuv_density::density_expand_v1::{lazy_sign, ExpandLimits, LazySignErrorV1};
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_sign::SignFailure;
use cftuv_density::{IBig, Rational};

#[test]
fn actual_owned_trigonometric_identity_reaches_geometric_undecidable() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let one = c.rational(&Rational::integer(IBig::ONE)).unwrap();
    let cosine = c.raw_cos(&one).unwrap();
    let sine = c.raw_sin(&one).unwrap();
    let cosine_square = c.mul(&[cosine.clone(), cosine]).unwrap();
    let sine_square = c.mul(&[sine.clone(), sine]).unwrap();
    let sum = c.add(&[cosine_square, sine_square]).unwrap();
    let identity = c.sub(&sum, &one).unwrap();
    let error = lazy_sign(
        &mut c,
        &identity,
        ExpandLimits {
            max_terms: 4096,
            max_polynomial_exponent: 16,
            max_rational_power_bits: 65536,
        },
    )
    .unwrap_err();
    assert_eq!(
        error,
        LazySignErrorV1::Sign(OwnedSignErrorV1::Sign(SignFailure::Undecidable))
    );
    let failure = cftuv_density::density_bind_v1::OwnedBindingFailureV1::Windows(
        cftuv_density::window_build::WindowBuildFailure::Callback(
            cftuv_density::density_laws_v1::LawsErrorV1::Sign(error),
        ),
    );
    assert_eq!(cftuv_density::density_bind_v1::binding_failure_text(&failure).unwrap(),
        "ReferenceGeometryError: Density A exact sign is not certified without generic factorization");
}
