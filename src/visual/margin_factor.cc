#include "../../include/visual/margin_factor.h"
#include "../../include/util/log.h"
#include "../../include/util/utils.h"

void ResidualBlockInfo::Evaluate() {
    residuals_.resize(cost_function_->num_residuals());
    vector<int> parameter_size = cost_function_->parameter_block_sizes();

    jacobians_.resize(parameter_size.size());
    raw_jacobians_ = new double*[parameter_size.size()];

    for (int i = 0; i < parameter_size.size(); i++) {
        jacobians_[i].resize(residuals_.size(), parameter_size[i]);
        raw_jacobians_[i] = jacobians_[i].data();
    }

    cost_function_->Evaluate(parameter_blocks_.data(), residuals_.data(), raw_jacobians_);

    if (loss_function_)
    {
        double residual_scaling_, alpha_sq_norm_;
        double sq_norm, rho[3];

        sq_norm = residuals_.squaredNorm();
        loss_function_->Evaluate(sq_norm, rho);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_    = 0.0;
        }
        else {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_    = alpha / sq_norm;
        }

        for (int i = 0; i < parameter_blocks_.size(); i++) {
            jacobians_[i] = sqrt_rho1_*(jacobians_[i] - alpha_sq_norm_*residuals_*(residuals_.transpose()*jacobians_[i]));
        }

        residuals_ *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo() {

    for (auto it = parameter_block_data_.begin(); it != parameter_block_data_.end(); ++it)
        delete[] it->second;

    for (int i = 0; i < (int)factors_.size(); i++) {
        delete[] factors_[i]->raw_jacobians_;
        delete factors_[i]->cost_function_;
        delete factors_[i];
    }
}

void 
MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info) {
    factors_.push_back(residual_block_info);

    const vector<double*>& parameter_block      = residual_block_info->parameter_blocks_;
    const vector<int>&     parameter_block_size = residual_block_info->cost_function_->parameter_block_sizes();
    const vector<int>&     drop_out             = residual_block_info->drop_set_;

    for (int i = 0; i < parameter_block.size(); i++) {
        parameter_block_size_[reinterpret_cast<long>(parameter_block[i])] = parameter_block_size[i];
    } 

    for (int i = 0; i < drop_out.size(); i++) {
        int idx = drop_out[i];
        parameter_block_idx_[reinterpret_cast<long>(parameter_block[idx])] = 0;
    }
    // cout << "iter output: " << parameter_block_idx_.size() << endl;
}

void 
MarginalizationInfo::preMarginalize() {
    for (auto it : factors_) {
        it->Evaluate();

        const vector<int>& block_sizes = it->cost_function_->parameter_block_sizes();
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
            long addr = reinterpret_cast<long>(it->parameter_blocks_[i]);
            int size = block_sizes[i];
            if (parameter_block_data_.find(addr) == parameter_block_data_.end()) {
                double *data = new double[size];
                memcpy(data, it->parameter_blocks_[i], sizeof(double) * size);
                parameter_block_data_[addr] = data;
            }
        }
    }
}

void 
MarginalizationInfo::marginalize() {
    int pose = 0;
    for (auto& pir : parameter_block_idx_) {
        pir.second = pose;
        pose +=  localSize(parameter_block_size_[pir.first]);
    }

    m_ = pose;
    for (const auto& pir : parameter_block_size_) {
        long addr = pir.first;
        if (parameter_block_idx_.find(addr) == parameter_block_idx_.end()) {
            parameter_block_idx_[addr] = pose;
            pose += localSize(pir.second);
        }
    }
    n_ = pose-m_;

    MatrixXd H;
    VectorXd b;

    H.resize(pose, pose);
    b.resize(pose);

    H.setZero();
    b.setZero();

    for (auto it : factors_) {
        auto& parameter_block = it->parameter_blocks_;
        auto& parameter_size  = it->cost_function_->parameter_block_sizes();
        
        const int N = parameter_block.size();
        for (int i = 0; i < N; i++) {
            long addr_i = reinterpret_cast<long>(parameter_block[i]);
            int  size_i = localSize(parameter_size[i]);
            int  idx_i  = parameter_block_idx_[addr_i];

            VectorXd r_i = it->residuals_;
            MatrixXd J_i = it->jacobians_[i].leftCols(size_i);
            for (int j = i; j < N; j++) {
                if (j == i) {
                    H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose()*J_i;
                }
                else {
                    long addr_j = reinterpret_cast<long>(parameter_block[j]);
                    int  size_j = localSize(parameter_size[j]);
                    int  idx_j  = parameter_block_idx_[addr_j];

                    MatrixXd J_j = it->jacobians_[j].leftCols(size_j);

                    H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose()*J_j;
                    H.block(idx_j, idx_i, size_j, size_i)  = H.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }

            b.segment(idx_i, size_i) += J_i.transpose()*r_i;
        }
    }

    // printMatrix("/home/ubuntu/catkin_ws/src/learn_vio/output/Hmatrix.csv", H, pose, pose);
    // printMatrix("/home/ubuntu/catkin_ws/src/learn_vio/output/bmatrix.csv", b, pose, 1);

    // shur complememt
    MatrixXd Hmm = 0.5 * (H.block(0, 0, m_, m_) + H.block(0, 0, m_, m_).transpose());
    SelfAdjointEigenSolver<MatrixXd> saes(Hmm);

    MatrixXd Hmm_inv = saes.eigenvectors()*Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();

    Eigen::VectorXd bmm = b.segment(0, m_);
    Eigen::MatrixXd Hmr = H.block(0, m_, m_, n_);
    Eigen::MatrixXd Hrm = H.block(m_, 0, n_, m_);
    Eigen::MatrixXd Hrr = H.block(m_, m_, n_, n_);
    Eigen::VectorXd brr = b.segment(m_, n_);
    
    MatrixXd Ht = Hrr - Hrm * Hmm_inv * Hmr;
    MatrixXd bt = brr - Hrm * Hmm_inv * bmm; 

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(Ht);
    VectorXd S     = VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    VectorXd S_inv = VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    VectorXd S_sqrt     = S.cwiseSqrt();
    VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    linear_jacobian_ = S_sqrt.asDiagonal()    *saes2.eigenvectors().transpose();
    linear_residual_ = S_inv_sqrt.asDiagonal()*saes2.eigenvectors().transpose()*bt;

    // printMatrix("/home/ubuntu/catkin_ws/src/learn_vio/output/Jmatrix.csv", linear_jacobian_, n_, n_);
    // printMatrix("/home/ubuntu/catkin_ws/src/learn_vio/output/ematrix.csv", linear_residual_, n_, 1);
}


bool MarginalizationInfo::check(MatrixXd& result_H, VectorXd& result_b) const {

}

vector<double*>
MarginalizationInfo::getParameterBlocks(unordered_map<long, double *> &addr_shift) {
    vector<double *> keep_block_addr;
    keep_block_size_.clear();
    keep_block_idx_.clear();
    keep_block_data_.clear();

    for (const auto &it : parameter_block_idx_)
    {
        if (it.second >= m_)
        {
            keep_block_size_.push_back(parameter_block_size_[it.first]);
            keep_block_idx_.push_back( parameter_block_idx_[it.first]);
            keep_block_data_.push_back(parameter_block_data_[it.first]);
            keep_block_addr_[reinterpret_cast<long>(addr_shift[it.first])] = reinterpret_cast<long>(parameter_block_data_[it.first]);
            keep_block_addr.push_back( addr_shift[it.first]);
        }
    }

    sum_block_size_ = std::accumulate(begin(keep_block_size_), end(keep_block_size_), 0);
    LOGE("[shift] keep parameter block: %d", sum_block_size_);

    return keep_block_addr;   
}

int
MarginalizationInfo::localSize(int size) const {
    return size == 7 ? 6 : size;
}

int 
MarginalizationInfo::globalSize(int size) const {
    return size == 6 ? 7 : size;
}


MarginalFactor::MarginalFactor(MarginalizationInfo* margin) :
    margin_(margin) {
    set_num_residuals(margin->n_);
    int cnt = 0;
    for (int it : margin_->keep_block_size_) {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }
    LOGE("[margin factor] parameter block size: %d(%d|%d)", cnt, margin_->sum_block_size_, margin_->n_);
}

bool MarginalFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {    
    int m = margin_->m_;
    int n = margin_->n_;

    // for (int i = 0; i < margin_->keep_block_size_.size(); i++) {
    //     const int   size = margin_->keep_block_size_[i];
    //     Map<VectorXd>       x0(margin_->keep_block_data_[i], size);
    //     Map<const VectorXd> x( parameters[i], size);

    //     cout << "i: " << i << "   " << x0.transpose() << endl;
    //     cout << "i: " << i << "   " << x.transpose()  << endl << endl;
    // }

    // assert(false);

    VectorXd dx(n);
    for (int i = 0; i < margin_->keep_block_size_.size(); i++) {

        const int   size = margin_->keep_block_size_[i];
        const int    idx = margin_->keep_block_idx_[i]-m;
        Map<VectorXd>       x0(margin_->keep_block_data_[i], size);
        Map<const VectorXd> x( parameters[i], size);
        
        if (size != 7) {
            dx.segment(idx, size) = x - x0;
        }
        else {            
            dx.segment(idx, 3) = x.head<3>() - x0.head<3>();

            Map<Quaterniond>       q0(margin_->keep_block_data_[i]+3);
            Map<const Quaterniond> q( parameters[i]+3);

            dx.segment(idx+3, 3) = (q0.inverse()*q).vec()*2;
            if ((q0.inverse()*q).w() < 0) { // theta is minus
                dx.segment(idx+3, 3) *= -1;
            }
        }
    }

    // for (int i = 0; i < n; i++) {
    //     fprintf(file, "%lf,", dx(0));
    // }
    // fprintf(file, "\n");

    Map<VectorXd> residual(residuals, n);
    residual = margin_->linear_residual_ + margin_->linear_jacobian_*dx;
    // for (int i = 0; i < n; i++) {
    //     fprintf(file, "%lf,", residuals[0]);
    // }
    // fprintf(file, "\n\n");

    if (jacobians) {
        for (int i = 0; i < margin_->keep_block_data_.size(); i++) {
            if (jacobians[i]) {
                int size  = margin_->keep_block_size_[i];
                int local = margin_->localSize(size);
                int idx   = margin_->keep_block_idx_[i]-m;
                
                Map<Matrix<double, Dynamic, Dynamic, RowMajor>> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local) = margin_->linear_jacobian_.middleCols(idx, local);
            }
        }
    }

    return true;
}


