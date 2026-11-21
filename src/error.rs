//! The crate's [`Error`] type.

use core::fmt;

use crate::geom::Point;

/// Why an operation failed.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// A coordinate lies outside `[-MAX_COORD, MAX_COORD]`
    /// ([`MAX_COORD`](crate::MAX_COORD)).
    CoordinateOutOfRange(Point),
    /// An obstacle id does not name a live obstacle.
    UnknownObstacle,
    /// A port id does not name a live port.
    UnknownPort,
    /// A port lies in the open interior of an obstacle, so no route can reach it.
    PortBlocked(Point),
    /// A port allows no direction.
    PortWithoutDirections(Point),
    /// No route exists between the two ports of a connector.
    NoRoute {
        /// The index of the connector in the request.
        connector: usize,
    },
    /// A cost product or sum does not fit into `u64`.
    CostOverflow,
    /// A net needs at least two ports.
    TooFewPorts,
    /// Two obstacle interiors overlap, which the triangulation-based routers do not
    /// support.
    OverlappingObstacles,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CoordinateOutOfRange(p) => write!(f, "coordinate out of range at {p}"),
            Self::UnknownObstacle => f.write_str("unknown obstacle id"),
            Self::UnknownPort => f.write_str("unknown port id"),
            Self::PortBlocked(p) => write!(f, "port {p} lies inside an obstacle"),
            Self::PortWithoutDirections(p) => write!(f, "port {p} allows no direction"),
            Self::NoRoute { connector } => write!(f, "no route for connector {connector}"),
            Self::CostOverflow => f.write_str("route cost overflows u64"),
            Self::TooFewPorts => f.write_str("a net needs at least two ports"),
            Self::OverlappingObstacles => f.write_str("obstacle interiors overlap"),
        }
    }
}

impl core::error::Error for Error {}
