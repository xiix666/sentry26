
/*
 *  Copyright (c) 2008--2011, Universitaet Bremen
 *  All rights reserved.
 *
 *  Author: Christoph Hertzberg <chtz@informatik.uni-bremen.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VECTVIEW_HPP_
#define VECTVIEW_HPP_

#include <Eigen/Core>

namespace MTK {

namespace internal {
	template<class Base, class T1, class T2>
	struct CovBlock {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF> ConstType;
	};

	template<class Base, class T1, class T2>
	struct CovBlock_ {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM> ConstType;
	};

	template<typename Base1, typename Base2, typename T1, typename T2>
	struct CrossCovBlock {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF> ConstType;
	};

	template<typename Base1, typename Base2, typename T1, typename T2>
	struct CrossCovBlock_ {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM> ConstType;
	};

	template<class scalar, int dim>
	struct VectviewBase {
		typedef Eigen::Matrix<scalar, dim, 1> matrix_type;
		typedef typename matrix_type::MapType Type;
		typedef typename matrix_type::ConstMapType ConstType;
	};

	template<class T>
	struct UnalignedType {
		typedef T type;
	};
}

template<class scalar, int dim>
class vectview : public internal::VectviewBase<scalar, dim>::Type {
	typedef internal::VectviewBase<scalar, dim> VectviewBase;
public:

	typedef typename VectviewBase::matrix_type matrix_type;

	typedef typename VectviewBase::Type base;

	explicit
	vectview(scalar* data, int dim_=dim) : base(data, dim_) {}

	vectview(matrix_type& m) : base(m.data(), m.size()) {}

	vectview(const vectview &v) : base(v) {}

	template<class Base>
	vectview(Eigen::VectorBlock<Base, dim> block) : base(&block.coeffRef(0), block.size()) {}
	template<class Base, bool PacketAccess>
	vectview(Eigen::Block<Base, dim, 1, PacketAccess> block) : base(&block.coeffRef(0), block.size()) {}

	using base::operator=;

	scalar* data() {return const_cast<scalar*>(base::data());}
};

template<class scalar, int dim>
class vectview<const scalar, dim> : public internal::VectviewBase<scalar, dim>::ConstType {
	typedef internal::VectviewBase<scalar, dim> VectviewBase;
public:

	typedef typename VectviewBase::matrix_type matrix_type;

	typedef typename VectviewBase::ConstType base;

	explicit
	vectview(const scalar* data, int dim_ = dim) : base(data, dim_) {}

	template<int options>
	vectview(const Eigen::Matrix<scalar, dim, 1, options>& m) : base(m.data()) {}

	template<int options, int phony>
	vectview(const Eigen::Matrix<scalar, 1, dim, options, phony>& m) : base(m.data()) {}

	vectview(vectview<scalar, dim> x) : base(x.data()) {}

	vectview(const base &x) : base(x) {}
	template<class Base>
	vectview(Eigen::VectorBlock<Base, dim> block) : base(&block.coeffRef(0)) {}
	template<class Base, bool PacketAccess>
	vectview(Eigen::Block<Base, dim, 1, PacketAccess> block) : base(&block.coeffRef(0)) {}

};

}

#endif
