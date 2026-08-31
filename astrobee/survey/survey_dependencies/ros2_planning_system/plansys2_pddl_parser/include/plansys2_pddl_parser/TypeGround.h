
#pragma once

#include <plansys2_msgs/Node.h>
#include <plansys2_msgs/Tree.h>

#include <plansys2_pddl_parser/Ground.h>

namespace parser { namespace pddl {

class TypeGround : public Ground {

public:

	TypeGround()
		: Ground() {}

	TypeGround( Lifted * l, const IntVec & p = IntVec() )
		: Ground( l, p ) {}

//	TypeGround( const TypeGround * tg )
//		: Ground( tg ) {}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::NodeSharedPtr getTree( plansys2_msgs::Tree & tree,
						const Domain & d,
						const std::vector<std::string> & replace = {} ) const override;

	void insert( Domain & d, const StringVec & v );

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

};

typedef std::vector< TypeGround * > TypeGroundVec;

} } // namespaces
