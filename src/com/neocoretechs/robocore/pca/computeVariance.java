package com.neocoretechs.robocore.pca;

import java.util.List;
/**
 * Use Eigenvector decomposition to perform principal component analysis on a point3 list.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2025 
 */
public class ComputeVariance {
	private boolean DEBUG;
	double variance1 = 0;
	double variance2 = 0;
	double variance3 = 0;
	Vector4d eigVec1, eigVec2, eigVec3;
	private Matrix3 m_covariance;
	private Point3f centroid;
	private double confidence;
	/**
	 * @return the variance1
	 */
	public double getVariance1() {
		return variance1;
	}
	/**
	 * @return the variance2
	 */
	public double getVariance2() {
		return variance2;
	}
	/**
	 * @return the variance3
	 */
	public double getVariance3() {
		return variance3;
	}
	/**
	 * @return the eigenvector1
	 */
	public Vector4d getEigvec1() {
		return eigVec1;
	}
	/**
	 * @return the eigenvector2
	 */
	public Vector4d getEigvec2() {
		return eigVec2;
	}
	/**
	 * @return the eigenvector3
	 */
	public Vector4d getEigvec3() {
		return eigVec3;
	}
	public Point3f getCentroid() {
		return centroid;
	}
	public double getConfidence() {
		return confidence;
	}
	/**
	 * @return the m_covariance
	 */
	public Matrix3 getM_covariance() {
		return m_covariance;
	}
	private Matrix3 computeCovariance(List<Point3f> window) {
	    int n = window.size();
	    double nd = (double) n;
	    Matrix3 cov = new Matrix3();
	    if (n < 2) {
	        return cov; // degenerate
	    }
	    // --- Compute centroid ---
	    double cx = 0, cy = 0, cz = 0;
	    for (Point3f p : window) {
	        cx += p.x();
	        cy += p.y();
	        cz += p.t();  // z = time
	    }
	    cx /= nd;
	    cy /= nd;
	    cz /= nd;
	    centroid = new Point3f((float)cx,(float)cy,(float)cz);
	    // --- Diagonal terms ---
	    double cxx = 0, cyy = 0, czz = 0;
	    for (Point3f p : window) {
	        double dx = p.x() - cx;
	        double dy = p.y() - cy;
	        double dz = p.t() - cz;
	        cxx += dx * dx;
	        cyy += dy * dy;
	        czz += dz * dz;
	    }
	    cov.set(0,0, cxx / nd);
	    cov.set(1,1, cyy / nd);
	    cov.set(2,2, czz / nd);
	    // --- Off-diagonal terms ---
	    double cxy = 0, cxz = 0, cyz = 0;
	    for (Point3f p : window) {
	        double dx = p.x() - cx;
	        double dy = p.y() - cy;
	        double dz = p.t() - cz;
	        cxy += dx * dy;
	        cxz += dx * dz;
	        cyz += dy * dz;
	    }
	    cov.set(0,1, cxy / nd);
	    cov.set(1,0, cxy / nd);
	    cov.set(0,2, cxz / nd);
	    cov.set(2,0, cxz / nd);
	    cov.set(1,2, cyz / nd);
	    cov.set(2,1, cyz / nd);
	    return cov;
	}
	/**
	 * Principal component analysis.
	 * Since the eigenvalues of the covariance matrix associated to the set of
	 * samples represent the proportions of the variances
	 * of the sample distribution
	 * when it is all said and done we have set the values of variance1,variance2,and variance3 along with
	 * eigenvector1, eigenvector2, and eigenvector3.
	 */
	public void leastVariance(List<Point3f> window){
		if(DEBUG) {
			System.out.println(this.getClass().getName()+" least_variance_direction...computing covariance, eigenvalues and eigenvectors");
		}
		m_covariance = computeCovariance(window);
		EigenvalueDecomposition eigenvalue_decomp = new EigenvalueDecomposition(m_covariance);
		double[] eigenvalues_vector = eigenvalue_decomp.getRealEigenvalues();
		int min_index = 0, max_index = 0, middle_index = 0;
		if(eigenvalues_vector[1] < eigenvalues_vector[min_index]) {
			min_index = 1;
		} else 
			if (eigenvalues_vector[1] > eigenvalues_vector[max_index]) {
				max_index = 1;
			}
		if(eigenvalues_vector[2] < eigenvalues_vector[min_index]) {
			min_index = 2;
		} else
			if (eigenvalues_vector[2] > eigenvalues_vector[max_index]) {
				max_index = 2;
			}
		while (middle_index==min_index || middle_index==max_index)
			middle_index++;

		variance1 = eigenvalues_vector[min_index];
		variance2 = eigenvalues_vector[middle_index];
		variance3 = eigenvalues_vector[max_index];
		if( DEBUG) {
			System.out.println(this.getClass().getName()+" least_variance_direction...variance1="+variance1+" variance2="+variance2+" variance3="+variance3);
		}
		Matrix3 eigenvectors_matrix = eigenvalue_decomp.getV();

		eigVec1 = new Vector4d(eigenvectors_matrix.get(0, min_index),eigenvectors_matrix.get(1, min_index),eigenvectors_matrix.get(2, min_index));
		eigVec2 = new Vector4d(eigenvectors_matrix.get(0, middle_index), eigenvectors_matrix.get(1, middle_index), eigenvectors_matrix.get(2, middle_index));
		eigVec3 = new Vector4d(eigenvectors_matrix.get(0, max_index), eigenvectors_matrix.get(1, max_index), eigenvectors_matrix.get(2, max_index));
		confidence = variance3 / (variance2 + variance1 + EigenvalueDecomposition.eps);
		if( DEBUG) {
			System.out.println(this.getClass().getName()+" least_variance_direction...eigenvector eigvec1="+eigVec1+" eigvec2="+eigVec2+" eigvec3="+eigVec3);
		}
	}

}
