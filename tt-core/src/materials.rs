//! Material configuration loader.
//!
//! Loads physical properties from YAML files, allowing easy configuration
//! of different rubbers, balls, and surfaces without recompiling.
//!
//! ## Directory Structure
//!
//! ```text
//! materials/
//! ├── rubbers/
//! │   ├── tenergy_05.yaml
//! │   ├── tenergy_09c.yaml
//! │   └── ...
//! ├── balls/
//! │   └── dhs_d40.yaml
//! └── surfaces/
//!     └── ittf_table.yaml
//! ```

use std::fs;
use std::path::{Path, PathBuf};

use crate::types::{BallProperties, RubberProperties, SurfaceProperties};

/// Error type for material loading operations.
#[derive(Debug)]
pub enum MaterialError {
    IoError(std::io::Error),
    ParseError(serde_yaml::Error),
    NotFound(String),
}

impl std::fmt::Display for MaterialError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MaterialError::IoError(e) => write!(f, "IO error: {}", e),
            MaterialError::ParseError(e) => write!(f, "YAML parse error: {}", e),
            MaterialError::NotFound(name) => write!(f, "Material not found: {}", name),
        }
    }
}

impl std::error::Error for MaterialError {}

impl From<std::io::Error> for MaterialError {
    fn from(err: std::io::Error) -> Self {
        MaterialError::IoError(err)
    }
}

impl From<serde_yaml::Error> for MaterialError {
    fn from(err: serde_yaml::Error) -> Self {
        MaterialError::ParseError(err)
    }
}

/// Material loader with configurable base directory.
pub struct MaterialLoader {
    base_path: PathBuf,
}

impl MaterialLoader {
    /// Create a new loader with the given base path.
    ///
    /// The base path should contain `rubbers/`, `balls/`, and `surfaces/` subdirectories.
    pub fn new<P: AsRef<Path>>(base_path: P) -> Self {
        Self {
            base_path: base_path.as_ref().to_path_buf(),
        }
    }

    /// Load a rubber by name (without .yaml extension).
    ///
    /// # Example
    /// ```ignore
    /// let loader = MaterialLoader::new("materials");
    /// let tenergy = loader.load_rubber("tenergy_09c")?;
    /// ```
    pub fn load_rubber(&self, name: &str) -> Result<RubberProperties, MaterialError> {
        let path = self.base_path.join("rubbers").join(format!("{}.yaml", name));
        if !path.exists() {
            return Err(MaterialError::NotFound(name.to_string()));
        }
        let contents = fs::read_to_string(&path)?;
        let props: RubberProperties = serde_yaml::from_str(&contents)?;
        Ok(props)
    }

    /// Load a ball by name.
    pub fn load_ball(&self, name: &str) -> Result<BallProperties, MaterialError> {
        let path = self.base_path.join("balls").join(format!("{}.yaml", name));
        if !path.exists() {
            return Err(MaterialError::NotFound(name.to_string()));
        }
        let contents = fs::read_to_string(&path)?;
        let props: BallProperties = serde_yaml::from_str(&contents)?;
        Ok(props)
    }

    /// Load a surface by name.
    pub fn load_surface(&self, name: &str) -> Result<SurfaceProperties, MaterialError> {
        let path = self
            .base_path
            .join("surfaces")
            .join(format!("{}.yaml", name));
        if !path.exists() {
            return Err(MaterialError::NotFound(name.to_string()));
        }
        let contents = fs::read_to_string(&path)?;
        let props: SurfaceProperties = serde_yaml::from_str(&contents)?;
        Ok(props)
    }

    /// List all available rubbers.
    pub fn list_rubbers(&self) -> Result<Vec<String>, MaterialError> {
        self.list_materials("rubbers")
    }

    /// List all available balls.
    pub fn list_balls(&self) -> Result<Vec<String>, MaterialError> {
        self.list_materials("balls")
    }

    /// List all available surfaces.
    pub fn list_surfaces(&self) -> Result<Vec<String>, MaterialError> {
        self.list_materials("surfaces")
    }

    fn list_materials(&self, subdir: &str) -> Result<Vec<String>, MaterialError> {
        let path = self.base_path.join(subdir);
        if !path.exists() {
            return Ok(vec![]);
        }

        let mut names = Vec::new();
        for entry in fs::read_dir(&path)? {
            let entry = entry?;
            let file_name = entry.file_name();
            let name = file_name.to_string_lossy();
            if name.ends_with(".yaml") {
                names.push(name.trim_end_matches(".yaml").to_string());
            }
        }
        names.sort();
        Ok(names)
    }
}

/// Paddle configuration combining blade and rubbers.
#[derive(Debug, Clone)]
pub struct PaddleConfig {
    pub name: String,
    pub blade: BladeProperties,
    pub forehand_rubber: RubberProperties,
    pub backhand_rubber: RubberProperties,
}

/// Properties of a blade (wooden paddle without rubbers).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BladeProperties {
    pub name: String,
    pub manufacturer: String,

    /// Blade thickness in mm
    pub thickness_mm: f64,

    /// Blade weight in grams (without rubbers)
    pub weight_g: f64,

    /// Stiffness factor (affects dwell time and feel)
    /// Higher = stiffer = less dwell, more direct
    pub stiffness: f64,

    /// Handle type (for reference)
    pub handle: String,
}

impl BladeProperties {
    /// Stiga Cybershape CC7 (Carbon Crystal 7) - penhold version
    // TODO: remove when MaterialLoader is implemented for blades
    pub fn stiga_cc7_penhold() -> Self {
        Self {
            name: "Cybershape CC7".to_string(),
            manufacturer: "Stiga".to_string(),
            thickness_mm: 5.8,
            weight_g: 88.0,
            stiffness: 0.85, // Carbon blades are stiffer
            handle: "penhold".to_string(),
        }
    }
}

impl Default for BladeProperties {
    fn default() -> Self {
        Self::stiga_cc7_penhold()
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::env;

    fn get_materials_path() -> PathBuf {
        // Try to find materials directory relative to manifest
        let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap_or_else(|_| ".".to_string());
        PathBuf::from(manifest_dir).join("..").join("materials")
    }

    #[test]
    fn test_load_existing_rubber() {
        let loader = MaterialLoader::new(get_materials_path());
        let result = loader.load_rubber("tenergy_05");

        assert!(result.is_ok(), "Should load tenergy_05: {:?}", result.err());
        let rubber = result.unwrap();
        assert_eq!(rubber.name, "Tenergy 05");
        assert!(rubber.restitution > 0.0 && rubber.restitution <= 1.0);
    }

    #[test]
    fn test_load_nonexistent_rubber() {
        let loader = MaterialLoader::new(get_materials_path());
        let result = loader.load_rubber("nonexistent_rubber_xyz");

        assert!(result.is_err());
        match result {
            Err(MaterialError::NotFound(name)) => {
                assert_eq!(name, "nonexistent_rubber_xyz");
            }
            _ => panic!("Expected NotFound error"),
        }
    }

    #[test]
    fn test_load_ball() {
        let loader = MaterialLoader::new(get_materials_path());
        let result = loader.load_ball("dhs_d40");

        assert!(result.is_ok(), "Should load dhs_d40: {:?}", result.err());
        let ball = result.unwrap();
        assert!(ball.mass > 0.0);
        assert!(ball.radius > 0.0);
    }

    #[test]
    fn test_load_surface() {
        let loader = MaterialLoader::new(get_materials_path());
        let result = loader.load_surface("ittf_table");

        assert!(result.is_ok(), "Should load ittf_table: {:?}", result.err());
    }

    #[test]
    fn test_list_rubbers() {
        let loader = MaterialLoader::new(get_materials_path());
        let result = loader.list_rubbers();

        assert!(result.is_ok());
        let rubbers = result.unwrap();
        assert!(rubbers.contains(&"tenergy_05".to_string()));
    }
}
