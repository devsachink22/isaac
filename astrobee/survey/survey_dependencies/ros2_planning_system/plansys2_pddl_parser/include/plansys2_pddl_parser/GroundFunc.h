
#pragma once

#include <plansys2_msgs/Node.h>
#include <plansys2_msgs/Tree.h>

#include <plansys2_pddl_parser/TypeGround.h>

namespace parser { namespace pddl {

template < typename T >
class GroundFunc : public TypeGround {

public:

	T value;

	GroundFunc()
		: TypeGround(), value( 0 ) {}

	GroundFunc( Lifted * l, const T & val = T( 0 ) )
		: TypeGround( l ), value( val ) {}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::NodeSharedPtr getTree( plansys2_msgs::Tree & tree,
						const Domain & d,
						const std::vector<std::string> & replace = {} ) const override;

	void print( std::ostream & stream ) const {
		stream << name << params << value << "\n";
	}

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

};

} } // namespaces
