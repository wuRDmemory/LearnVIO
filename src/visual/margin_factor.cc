#include "../../include/visual/margin_factor.h"

void ResidualBlockInfo::Evaluate() {
    residuals_.resize(cost_function_->num_residuals());
    vector<int> parameter_size = cost_function_->parameter_block_sizes();

    jacobians_.resize(parameter_size.size());
    double** raw_jacobians = new double*[parameter_size.size()];

    for (int i = 0; i < parameter_size.size(); i++) {
        jacobians_[i].resize(residuals_.size(), parameter_size[i]);
        raw_jacobians[i] = jacobians_[i].data();
    }

    double* residual = residuals_.data();
    cost_function_->Evaluate(parameter_blocks_.data(), residuals_.data(), raw_jacobians);

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
    for (auto& pir : parameter_block_data_) {
        delete(pir.second);
    }
    parameter_block_data_.clear();
    parameter_block_size_.clear();
    parameter_block_idx_.clear();

    for (int i = 0; i < keep_block_data_.size(); i++) {
        delete(keep_block_data_[i]);
    }
    keep_block_data_.clear();

    for (int i = 0; i < factors_.size(); i++) {
        delete(factors_[i]);
    }
    factors_.clear();
}

void 
MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info) {
    factors_.push_back(residual_block_info);

    auto& parameter_block      = residual_block_info->parameter_blocks_;
    auto& parameter_block_size = residual_block_info->cost_function_->parameter_block_sizes();
    auto& drop_out             = residual_block_info->drop_set_;

    for (int i = 0; i < parameter_block.size(); i++) {
        parameter_block_size_[reinterpret_cast<long>(parameter_block[i])] = parameter_block_size[i];
    } 

    for (int i = 0; i < drop_out.size(); i++) {
        parameter_block_idx_[reinterpret_cast<long>(parameter_block[i])] = 0;
    }    
}

void 
MarginalizationInfo::preMarginalize() {
    for (auto it : factors_) {
        it->Evaluate();

        vector<int> block_sizes = it->cost_function_->parameter_block_sizes();
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
    for (auto& pir : parameter_block_idx_) {
        long addr = pir.first;
        parameter_block_idx_[addr] = m_;
        
        m_ +=  localSize(parameter_block_size_[addr]);
    }

    n_ = m_;
    for (auto& pir : parameter_block_size_) {
        long addr = pir.first;
        if (!parameter_block_idx_.count(addr)) {
            parameter_block_idx_[addr] = n_;
            n_ += localSize(pir.second);
        }
    }
    sum_block_size_ = n_;
    n_ -= m_;

    MatrixXd H;
    VectorXd b;

    H.resize(sum_block_size_, sum_block_size_);
    b.resize(sum_block_size_);

    H.setZero();
    b.setZero();

    for (auto& it : factors_) {
        auto& parameter_block = it->parameter_blocks_;
        auto& parameter_size  = it->cost_function_->parameter_block_sizes();
        
        const int N = parameter_block.size();
        for (int i = 0; i < N; i++) {
            long addr_i = reinterpret_cast<long>(parameter_block[i]);
            int  size_i = localSize(parameter_size[i]);
            int  idx_i  = parameter_block_idx_[i];

            VectorXd r_i = it->residuals_;
            MatrixXd J_i = it->jacobians_[i].topLeftCorner(size_i, size_i);
            for (int j = i; j < N; j++) {
                if (j == i) {
                    H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose()*J_i;
                }
                else {
                    long addr_j = reinterpret_cast<long>(parameter_block[j]);
                    int  size_j = localSize(parameter_size[j]);
                    int  idx_j  = parameter_block_idx_[j];

                    MatrixXd J_j = it->jacobians_[j].topLeftCorner(size_j, size_j);

                    H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose()*J_j;
                    H.block(idx_j, idx_i, size_i, size_j) += H.block(idx_i, idx_j, size_i, size_j);

                }
            }

            b.segment(idx_i, size_i) += J_i.transpose()*r_i;
        }
    }

    // shur complememt
    MatrixXd Hmm = 0.5 * (H.block(0, 0, m_, m_) + H.block(0, 0, m_, m_).transpose());
    SelfAdjointEigenSolver<MatrixXd> saes(Hmm);

    MatrixXd Hmm_inv = saes.eigenvectors()*Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();

    Eigen::VectorXd bmm = b.segment(0, m_);
    Eigen::MatrixXd Hmr = H.block(0, m_, m_, n_);
    Eigen::MatrixXd Hrm = H.block(m_, 0, n_, m_);
    Eigen::MatrixXd Hrr = H.block(m_, m_, n_, n_);
    Eigen::VectorXd brr = b.segment(m_, n_);
    
    H = Hrr - Hrm * Hmm_inv * Hmr;
    b = brr - Hrm * Hmm_inv * bmm; 

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H);
    VectorXd S     = VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    VectorXd S_inv = VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    VectorXd S_sqrt     = S.cwiseSqrt();
    VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    linear_jacobian_ = S_sqrt.asDiagonal()    *saes2.eigenvectors().transpose();
    linear_residual_ = S_inv_sqrt.asDiagonal()*saes2.eigenvectors().transpose()*b;
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
            keep_block_addr.push_back( addr_shift[it.first]);
        }
    }
    // sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

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
    for (int it : margin_->keep_block_size_) {
        mutable_parameter_block_sizes()->push_back(it);
    }
}

bool MarginalFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    int m = margin_->m_;
    int n = margin_->n_;
    
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
            dx.segment(idx, 3) = x.segment(0, 3) - x0.segment(0, 3);

            Map<Quaterniond>       q0(margin_->keep_block_data_[i]+3);
            Map<const Quaterniond> q( parameters[i]+3);

            dx.segment(idx+3, 3) = (q0.inverse()*q).vec()*2;
            if ((q0.inverse()*q).w() < 0) { // theta is minus
                dx.segment(idx+3, 3) *= -1;
            }
        }
    }

    Map<VectorXd> residual(residuals, n);
    residual = margin_->linear_residual_ + margin_->linear_jacobian_*dx;

    if (jacobians) {
        for (int i = 0; i < margin_->keep_block_data_.size(); i++) {
            if (jacobians[i]) {
                int size  = margin_->keep_block_size_[i];
                int local = margin_->localSize(size);
                int idx   = margin_->keep_block_idx_[i]-m;
                
                Map<MatrixXd> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local) = margin_->linear_jacobian_.middleCols(idx, local);
            }
        }
    }

    return true;
}


