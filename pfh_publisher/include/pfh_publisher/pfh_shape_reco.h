#ifndef PFH_SHAPE_RECOGNITION_H
#define PFH_SHAPE_RECOGNITION_H

#include <pcl/point_cloud.h>
#include <pcl/features/pfh.h>

template<typename T>
class PFHShapeReco {
    typedef boost::shared_ptr <pcl::PointCloud<T>> CloudPtr;

    // A model: a name and a histogram
    struct sModel {
        std::string modelName;
        std::vector<float> modelHist;
    };

public:
    PFHShapeReco(int nbrPointsPairInHist = 10000, int nbrBinsAngles = 5, int nbrBinsDist = 5);

    ~PFHShapeReco();

    // Set parameters
    void SetParams();

    // Get parameters
    void GetParams(int &nbrPointsPairInHist, int &nbrBinsAngles, int &nbrBinsDist);

    // Set point clouds to use, and estimate object size
    void ComputePFH(float coef, CloudPtr &cloudPtr, pcl::PointCloud<pcl::Normal>::Ptr &normalsCloudPtr);

    // Get object size estimated when loading the point clouds
    float GetMaxDist();

    // Compute distance between current object's histogram and the histograms in the database
    void IdentifyObject();

    // Get the list of scores, for each model name
    std::map<std::string, float> GetIdentificationScores();

    // Get histogram of the current object
    std::vector<float> &GetObjectModel();

    // Add model of the current object to database
    void AddModelToDatabase(std::string modelName);

    // Clear database
    void ClearDatabase();

    // Save database to file
    void SaveDatabase(std::string fileName);

    // Load database from file
    void LoadDatabase(std::string fileName);

protected:
    // Nbr of point pairs used to build histograms
    int _nbrPointsPairInHist;
    // Nbr of bins for angle features (f1,f2,f3) and distance feature (f4)
    int _nbrBinsAngles, _nbrBinsDist;
    // Identified object label and score
    std::string object_label_;
    float object_score_;

    // Max number of levels in the pyramid
    int _iMaxPyramidDepth;
    // Largest scale of the pyramid (in meters)
    float _fMinScale;

    void SetObjectModel(const std::vector<float> model);

private:
    // Histogram of the current object
    std::vector<float> _objectModel;

    // Compute histogram of the current object
    void _ComputeGlobalModel();

    // Estimate size of the current object
    void _calcMaxDist();

    void _calcMaxDist2(float coef);

    float _maxDist;


    // Point clouds
    CloudPtr _cloudPtr;
    pcl::PointCloud<pcl::Normal>::Ptr _normalsCloudPtr;
    size_t _cloudSize;

    // Nbr of models in the database
    unsigned int _nbrModels;
    // Factors to discretize the features
    float _fAngleFactor, _fDistFactor;
    // Size of the histogram
    size_t _sizeHist;
    // Multipliers used to move in the histogram (based on number of bins)
    unsigned int _base1, _base2, _base3, _base4;
    // Database of histograms
    std::vector<struct sModel> _modelsDatabase;
    // Map with model names and scores, after identification
    std::map<std::string, float> _identificationScores;
};

#include "pfh_shape_reco.hpp"

#endif // PFH_SHAPE_RECOGNITION_H
