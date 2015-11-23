//! Classes for handling linear algebra.
//!
//! We keep it simple, here, just for our purposes.

use std::ops::{Add, Index, IndexMut, Mul};

/// A 3x3 rotation matrix.
///
/// These can be multiplied by `Vector3`:
///
/// ```
/// use georef::linalg::{Matrix3, Vector3};
/// let matrix = Matrix3::identity();
/// let vector = Vector3(1.0, 2.0, 3.0);
/// let vector2 = matrix * vector;
/// assert_eq!(Vector3(1.0, 2.0, 3.0), vector2);
/// ```
#[derive(Clone, Copy, Debug, Default)]
pub struct Matrix3 {
    data: [f64; 9],
}

impl Matrix3 {
    /// Creates a new matrix with all zeros.
    ///
    /// # Examples
    ///
    /// ```
    /// use georef::linalg::Matrix3;
    /// let matrix = Matrix3::new();
    /// ```
    pub fn new() -> Matrix3 {
        Default::default()
    }

    /// Creates a new rotation matrix with ones on the diagonals.
    ///
    /// # Examples
    ///
    /// ```
    /// use georef::linalg::Matrix3;
    /// let matrix = Matrix3::identity();
    /// assert_eq!(1.0, matrix[(0, 0)]);
    /// assert_eq!(0.0, matrix[(0, 1)]);
    /// assert_eq!(0.0, matrix[(0, 2)]);
    /// assert_eq!(0.0, matrix[(1, 0)]);
    /// assert_eq!(1.0, matrix[(1, 1)]);
    /// assert_eq!(0.0, matrix[(1, 2)]);
    /// assert_eq!(0.0, matrix[(2, 0)]);
    /// assert_eq!(0.0, matrix[(2, 1)]);
    /// assert_eq!(1.0, matrix[(2, 2)]);
    /// ```
    pub fn identity() -> Matrix3 {
        Matrix3 { data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] }
    }
}

impl Mul<Vector3> for Matrix3 {
    type Output = Vector3;
    fn mul(self, rhs: Vector3) -> Self::Output {
        Vector3(self[(0, 0)] * rhs.0 + self[(0, 1)] * rhs.1 + self[(0, 2)] * rhs.2,
                self[(1, 0)] * rhs.0 + self[(1, 1)] * rhs.1 + self[(1, 2)] * rhs.2,
                self[(2, 0)] * rhs.0 + self[(2, 1)] * rhs.1 + self[(2, 2)] * rhs.2)
    }
}

impl Index<(usize, usize)> for Matrix3 {
    type Output = f64;
    fn index(&self, (r, c): (usize, usize)) -> &Self::Output {
        &self.data[r * 3 + c]
    }
}

impl IndexMut<(usize, usize)> for Matrix3 {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut Self::Output {
        &mut self.data[r * 3 + c]
    }
}

/// A three-element vector of floats.
///
/// These can be added:
///
/// ```
/// use georef::linalg::Vector3;
/// let vector = Vector3(1.0, 2.0, 3.0) + Vector3(4.0, 5.0, 6.0);
/// assert_eq!(Vector3(5.0, 7.0, 9.0), vector);
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3(pub f64, pub f64, pub f64);

impl Add for Vector3 {
    type Output = Vector3;
    fn add(self, rhs: Vector3) -> Self::Output {
        Vector3(self.0 + rhs.0, self.1 + rhs.1, self.2 + rhs.2)
    }
}
