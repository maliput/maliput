//! Type-specific identifiers for maliput elements.
//!
//! These identifiers provide type-safe IDs for different road network elements,
//! preventing accidental mixing of IDs from different element types.

use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

/// A type-specific identifier that associates a string ID with a particular type.
///
/// This provides compile-time safety to ensure that, for example, a `LaneId`
/// cannot be accidentally used where a `SegmentId` is expected.
///
/// # Type Parameter
///
/// * `T` - The type this identifier is associated with (used for type safety only)
///
/// # Examples
///
/// ```
/// use maliput::api::{TypeSpecificIdentifier, Lane, Segment};
///
/// // These are different types and cannot be mixed
/// type LaneId = TypeSpecificIdentifier<dyn Lane>;
/// type SegmentId = TypeSpecificIdentifier<dyn Segment>;
///
/// let lane_id = LaneId::new("lane_1".to_string());
/// let segment_id = SegmentId::new("segment_1".to_string());
///
/// // This would be a compile error:
/// // let wrong: LaneId = segment_id;
/// ```
#[derive(Debug, Clone)]
pub struct TypeSpecificIdentifier<T: ?Sized> {
    id: String,
    _marker: PhantomData<T>,
}

impl<T: ?Sized> TypeSpecificIdentifier<T> {
    /// Creates a new identifier from a string.
    ///
    /// # Panics
    ///
    /// Panics if the string is empty.
    pub fn new(id: String) -> Self {
        assert!(!id.is_empty(), "Identifier string cannot be empty");
        Self {
            id,
            _marker: PhantomData,
        }
    }

    /// Returns the string representation of this identifier.
    pub fn string(&self) -> &str {
        &self.id
    }
}

impl<T: ?Sized> PartialEq for TypeSpecificIdentifier<T> {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl<T: ?Sized> Eq for TypeSpecificIdentifier<T> {}

impl<T: ?Sized> Hash for TypeSpecificIdentifier<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl<T: ?Sized> fmt::Display for TypeSpecificIdentifier<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.id)
    }
}

impl<T: ?Sized> PartialOrd for TypeSpecificIdentifier<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: ?Sized> Ord for TypeSpecificIdentifier<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.id.cmp(&other.id)
    }
}

// Type aliases for common identifiers
// Note: We use trait objects as the type parameter since the actual types are traits

/// Identifier for a [`RoadGeometry`](super::RoadGeometry).
pub type RoadGeometryId = TypeSpecificIdentifier<dyn super::RoadGeometry>;

/// Identifier for a [`Junction`](super::Junction).
pub type JunctionId = TypeSpecificIdentifier<dyn super::Junction>;

/// Identifier for a [`Segment`](super::Segment).
pub type SegmentId = TypeSpecificIdentifier<dyn super::Segment>;

/// Identifier for a [`Lane`](super::Lane).
pub type LaneId = TypeSpecificIdentifier<dyn super::Lane>;

/// Identifier for a [`BranchPoint`](super::BranchPoint).
pub type BranchPointId = TypeSpecificIdentifier<dyn super::BranchPoint>;

#[cfg(test)]
mod tests {
    use super::*;

    // Dummy type for testing
    #[derive(Debug)]
    struct TestType;

    #[test]
    fn test_identifier_creation() {
        let id: TypeSpecificIdentifier<TestType> = TypeSpecificIdentifier::new("test_id".to_string());
        assert_eq!(id.string(), "test_id");
    }

    #[test]
    #[should_panic(expected = "Identifier string cannot be empty")]
    fn test_empty_identifier_panics() {
        let _id: TypeSpecificIdentifier<TestType> = TypeSpecificIdentifier::new(String::new());
    }

    #[test]
    fn test_identifier_equality() {
        let id1: TypeSpecificIdentifier<TestType> = TypeSpecificIdentifier::new("same".to_string());
        let id2: TypeSpecificIdentifier<TestType> = TypeSpecificIdentifier::new("same".to_string());
        let id3: TypeSpecificIdentifier<TestType> = TypeSpecificIdentifier::new("different".to_string());

        assert_eq!(id1, id2);
        assert_ne!(id1, id3);
    }

    #[test]
    fn test_identifier_hash() {
        use std::collections::HashSet;

        let mut set: HashSet<TypeSpecificIdentifier<TestType>> = HashSet::new();
        set.insert(TypeSpecificIdentifier::new("id1".to_string()));
        set.insert(TypeSpecificIdentifier::new("id2".to_string()));
        set.insert(TypeSpecificIdentifier::new("id1".to_string())); // duplicate

        assert_eq!(set.len(), 2);
    }
}
