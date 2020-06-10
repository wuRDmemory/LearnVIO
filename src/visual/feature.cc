#include "../../include/visual/feature.h"
#include "../../include/log.h"

FeatureManager::FeatureManager() {
    all_ftr_.clear();
}

FeatureManager::~FeatureManager() {
    ;
}

bool FeatureManager::addNewFeatures(const Image_Type& image_data, int frame_id) {
    vector<int> track_ftr_ids;
    track_ftr_ids.reserve(100);

    for (auto& id_data : image_data) {
        const int               id = id_data.first;
        const Elem_Type&     cdata = id_data.second;
        const vector<double>& data = cdata.second;

        assert(data.size() == 7);

        Vector3f f(data[0], data[1], data[2]);
        Vector2f uv(data[3], data[4]);
        Vector2f velocity(data[5], data[6]);

        if (all_ftr_.count(id)) {
            // already exist
            all_ftr_[id]->addFrame(f, uv);
            track_ftr_ids.push_back(id);
        }
        else {
            // not exist
            Feature* new_ftr = new Feature(id, frame_id, f, uv);
            all_ftr_.insert(make_pair(id, new_ftr));
        }
    }

    LOGD("[FM add ftr] track feature cnt : %d", (int)track_ftr_ids.size());

    if (frame_id < 2 || track_ftr_ids.size() < 20) {
        // new key frame        
        return true;
    }

    float parallax_sum = 0;
    int   parallax_num = 0;
    for (auto& pr : all_ftr_) {
        int       id = pr.first;  // feature id
        Feature* ftr = pr.second; // feature

        if (   ftr->getRefFrameId()             <  frame_id-2
            && ftr->getRefFrameId()+ftr->size() >= frame_id ) {
            parallax_sum += computeParallax(ftr, frame_id);
            parallax_num++;
        }
    }

    if (parallax_num == 0) {
        // not enough good feature in feature manager
        return true;
    }
    else {
        // enough good feature
        parallax_sum /= parallax_num;
        parallax_sum *= FOCAL_LENGTH;

        LOGI("[FM add ftr] parallax : %f", parallax_sum);
        return parallax_sum >= 10;
    }

    return true;
}

float FeatureManager::computeParallax(Feature* ftr, int frame_id) {
    int ref_index   = ftr->ref_frame_id_;
    Vector3f& ftr_i = ftr->vis_fs_[frame_id - ref_index - 1];
    Vector3f& ftr_j = ftr->vis_fs_[frame_id - ref_index - 2];

    float dx = ftr_i.x()/ftr_i.z() - ftr_j.x()/ftr_j.z();
    float dy = ftr_i.y()/ftr_i.z() - ftr_j.y()/ftr_j.z();

    return max(0.0f, sqrtf(dx*dx + dy*dy));
}

