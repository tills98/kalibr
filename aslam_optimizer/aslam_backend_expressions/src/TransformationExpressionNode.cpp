#include <aslam/backend/TransformationExpressionNode.hpp>
#include <sm/kinematics/transformations.hpp>
#include <Eigen/Dense>
#include <typeinfo>

namespace aslam {
  namespace backend {
    
    ////////////////////////////////////////////
    // TransformationExpressionNode: The Super Class
    ////////////////////////////////////////////
    TransformationExpressionNode::TransformationExpressionNode(){}

    TransformationExpressionNode::~TransformationExpressionNode(){}

    Eigen::Matrix4d TransformationExpressionNode::toTransformationMatrix(){
      std::cout << "TransformationExpressionNode::toTransformationMatrix " << typeid(this).name() << std::endl;
      Eigen::Matrix4d x;
      Eigen::Matrix4d (TransformationExpressionNode::*funcPtr)() =
            &TransformationExpressionNode::toTransformationMatrixImplementation;

      if (funcPtr) {
        std::cout << "yes it should exist " << typeid(funcPtr).name() << std::endl;
        Eigen::Matrix4d x = (this->*funcPtr)();
        std::cout << "okay it does things..." << std::endl;
      } else {
        std::cout << "no it not exist" << std::endl;
      }

      try {
        std::cout << "TransformationExpressionNode::toTransformationMatrix inside try " << std::endl;
      	x = toTransformationMatrixImplementation();
      } catch (Exception &e) {
      	std::cout << "Oups..." << std::endl;
      }
      std::cout << "after TransformationExpressionNode::toTransformationMatrix " << std::endl;
      return x;
    }

    void TransformationExpressionNode::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      evaluateJacobiansImplementation(outJacobians);
    }      

    void TransformationExpressionNode::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      SM_ASSERT_EQ_DBG(Exception, applyChainRule.cols(), 6, "The chain rule matrix must have 6 columns");
      evaluateJacobiansImplementation(outJacobians, applyChainRule);
    }
      
    void TransformationExpressionNode::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      getDesignVariablesImplementation(designVariables);
    }


    /////////////////////////////////////////////////
    // TransformationExpressionNodeMultiply: A container for C1 * C2
    /////////////////////////////////////////////////

    TransformationExpressionNodeMultiply::TransformationExpressionNodeMultiply(boost::shared_ptr<TransformationExpressionNode> lhs, boost::shared_ptr<TransformationExpressionNode> rhs):
      _lhs(lhs), _rhs(rhs)
    {
      _T_lhs = _lhs->toTransformationMatrix();
      _T_rhs = _rhs->toTransformationMatrix();
    }

    TransformationExpressionNodeMultiply::~TransformationExpressionNodeMultiply(){}

    void TransformationExpressionNodeMultiply::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    Eigen::Matrix4d TransformationExpressionNodeMultiply::toTransformationMatrixImplementation()
    {
      std::cout << "TransformationExpressionNodeMultiply::toTransformationMatrixImplementation " << std::endl;
      std::cout << "_lhs" << _lhs << std::endl;
      std::cout << "_rhs"  << _rhs << std::endl;
      _T_lhs = _lhs->toTransformationMatrix();
      std::cout << "_T_lhs" << std::endl;
      _T_rhs = _rhs->toTransformationMatrix();
      std::cout << "_T_rhs" << std::endl;

      return  _T_lhs * _T_rhs;
    }

    void TransformationExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {	
      _rhs->evaluateJacobians(outJacobians,sm::kinematics::boxTimes(_T_lhs));
      _lhs->evaluateJacobians(outJacobians);
    }

 
    void TransformationExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {	
      _rhs->evaluateJacobians(outJacobians, (applyChainRule * sm::kinematics::boxTimes(_T_lhs)));
      _lhs->evaluateJacobians(outJacobians, applyChainRule);
    }


    ////////////////////////////////////////////////////
    /// TransformationExpressionNodeInverse: A container for T^-1
    ////////////////////////////////////////////////////
    
    TransformationExpressionNodeInverse::TransformationExpressionNodeInverse(boost::shared_ptr<TransformationExpressionNode> dvTransformation) : _dvTransformation(dvTransformation)
    {
      _T = _dvTransformation->toTransformationMatrix().inverse();
    }
    
    TransformationExpressionNodeInverse::~TransformationExpressionNodeInverse(){}

    Eigen::Matrix4d TransformationExpressionNodeInverse::toTransformationMatrixImplementation()
    {
      std::cout << "TransformationExpressionNodeInverse::toTransformationMatrixImplementation" << std::endl;
      _T = _dvTransformation->toTransformationMatrix().inverse();
      return  _T;
    }

    void TransformationExpressionNodeInverse::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _dvTransformation->evaluateJacobians(outJacobians, -sm::kinematics::boxTimes(_T));
    }


    void TransformationExpressionNodeInverse::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _dvTransformation->evaluateJacobians(outJacobians, (applyChainRule * -sm::kinematics::boxTimes(_T)));
    }

    void TransformationExpressionNodeInverse::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _dvTransformation->getDesignVariables(designVariables);
    }

      TransformationExpressionNodeConstant::TransformationExpressionNodeConstant(const Eigen::Matrix4d & T) : _T(T) {}
      TransformationExpressionNodeConstant::~TransformationExpressionNodeConstant(){}

    
      Eigen::Matrix4d TransformationExpressionNodeConstant::toTransformationMatrixImplementation(){
        std::cout << "TransformationExpressionNodeConstant::toTransformationMatrixImplementation" << std::endl;
        return _T;
      }
  void TransformationExpressionNodeConstant::evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */) const{}
  void TransformationExpressionNodeConstant::evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */, const Eigen::MatrixXd & /* applyChainRule */) const{}
  void TransformationExpressionNodeConstant::getDesignVariablesImplementation(DesignVariable::set_t & /* designVariables */) const{}


  } // namespace backend
} // namespace aslam
