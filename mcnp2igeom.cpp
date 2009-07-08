#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <cctype>
#include <vector>
#include <cmath>
#include <map>

#include <cassert>

//#include "MCNPInputDeck.hpp"
#include "iGeom.h"

std::string& strlower( std::string& str ){
  // convert to lowercase
  for(size_t i = 0; i < str.length(); ++i){
    str[i] = tolower( str.at(i) );
  }
  return str;
}

typedef std::vector< std::string > token_list_t;

template < class T >
std::ostream& operator<<( std::ostream& out, const std::vector<T>& list ){
  out << "[";
  for(typename std::vector<T>::const_iterator i = list.begin(); i!=list.end(); ++i){
    out << *i << "|";
  }
  out << "\b]" << std::endl;
  return out;
}

double makedouble( const std::string& token ){
  const char* str = token.c_str();
  char* end;
  double ret = strtod(str, &end);
  if( end != str+token.length() ){
    std::cerr << "Warning: string [" << token << "] did not convert to double as expected." << std::endl;
  }
  return ret;
}

int makeint( const std::string& token ){
  const char* str = token.c_str();
  char* end;
  int ret = strtol(str, &end, 10);
  if( end != str+token.length() ){
    std::cerr << "Warning: string [" << token << "] did not convert to int as expected." << std::endl;
  }
  return ret;
}

bool isblank( const std::string& line ){
  return (line=="" || line.find_first_not_of(" ") == line.npos );
}

class CellCard;
class SurfaceCard;
class LineExtractor;
class InputDeck;

class Card{

protected:
  Card(){}

};

class CellCard : public Card {

public:
  enum geom_token_t {INTERSECT, UNION, COMPLEMENT, LPAREN, RPAREN, CELLNUM, SURFNUM};
  typedef std::pair<enum geom_token_t, int> geom_list_entry_t;
  typedef std::vector<geom_list_entry_t> geom_list_t;

protected:
  int ident;
  geom_list_t geom;
  token_list_t data;

  static geom_list_entry_t make_geom_entry(geom_token_t t, int param = 0){
    return std::make_pair(t, param);
  }

  static bool is_num_token( geom_list_entry_t t ){
    return t.first == CELLNUM || t.first == SURFNUM;
  }

  static bool is_op_token( geom_list_entry_t t ){
    return t.first == COMPLEMENT || t.first == UNION || t.first == INTERSECT;
  }

  static int operator_priority( geom_list_entry_t t ){
    switch(t.first){
    case COMPLEMENT: return 3;
    case INTERSECT:  return 2;
    case UNION:      return 1;
    default:
      throw std::runtime_error("queried operator priority for a non-operator token");
    }
  }

  /**
   * Build the geom list as part of cell construction.
   * Each item in the list will be a string; either " ", ":", or "#", indicating
   * operators, or parentheses, or numbers indicating surface or cell identities.
   *
   * @param The list of geometry tokens in the input file, as a list of strings that were 
   *        separated by white space in the original file.
   */
  void retokenize_geometry( const token_list_t& tokens ){
    for(token_list_t::const_iterator i = tokens.begin(); i!=tokens.end(); ++i){
      const std::string& token = *i;
      
      size_t j = 0;
      while( j < token.length() ){

	char cj = token.at(j);

	switch(cj){
	  
	  // the following macro pushes an intersect token onto the geom list
	  // if the end of that list indicates that one is needed
#define IMPLICIT_INTERSECT() do{				\
	    if(geom.size()){					\
	      geom_list_entry_t &t = geom.at(geom.size()-1);	\
	      if( is_num_token(t) || t.first == RPAREN ){	\
		geom.push_back( make_geom_entry( INTERSECT ) );	\
	      }}} while(0) 
	  
	case '(': 
	  IMPLICIT_INTERSECT();
	  geom.push_back(make_geom_entry(LPAREN)); j++;
	  break;
	  
	case ')':
	  geom.push_back(make_geom_entry(RPAREN)); j++;
	  break;
	  
	case '#':
	  IMPLICIT_INTERSECT();
	  geom.push_back(make_geom_entry(COMPLEMENT)); j++; 
	  break;
	  
	case ':':		
	  geom.push_back(make_geom_entry(UNION)); j++; 
	  break;
	  
	default: // a number
	  // the number refers to a cell if the previous token is a complement
	  bool is_cell = geom.size() && ((geom.at(geom.size()-1)).first == COMPLEMENT);
	  IMPLICIT_INTERSECT();
	  assert(isdigit(cj) || cj == '+' || cj == '-' );
	  size_t end = token.find_first_not_of("1234567890-+",j);
	  assert(j != end);
	  int num = strtol( std::string(token, j, end-j).c_str(), NULL, 10 );
	  geom.push_back( make_geom_entry( is_cell ? CELLNUM : SURFNUM, num ));
	  j += (end-j);
	  break;
#undef IMPLICIT_INTERSECT

	}
	
      } 
    }
    
    std::cout << tokens << " -> " << geom << std::endl;
    
  }


  /**
   * The final step of geometry parsing: convert the geometry list to RPN, which
   * greatly simplifies the process of evaluating the geometry later.  This function
   * uses the shunting yard algorithm.  For more info consult
   * http://en.wikipedia.org/wiki/Shunting_yard_algorithm
   */
  void shunt_geometry( ){
    geom_list_t geom_copy( geom );
    geom.clear();

    geom_list_t stack;
    for(geom_list_t::iterator i = geom_copy.begin(); i!=geom_copy.end(); ++i){
      geom_list_entry_t token = *i;
      if( is_num_token(token) ){
	geom.push_back(token);
      }
      else if( is_op_token(token) ){

	while(stack.size()){
	  geom_list_entry_t& stack_top = stack.back();
	  if( is_op_token(stack_top) && operator_priority(stack_top) > operator_priority(token) ){
	    geom.push_back(stack_top);
	    stack.pop_back();
	  }
	  else{
	    break;
	  }
	}
	stack.push_back(token);
      }
      else if( token.first == LPAREN ){
	stack.push_back(token);
      }
      else if( token.first == RPAREN ){
	while( stack.back().first != LPAREN ){
	  geom.push_back( stack.back() );
	  stack.pop_back();
	}
	stack.pop_back(); // remove the LPAREN
      }
    }
    while( stack.size() ){
      geom.push_back( stack.back() );
      stack.pop_back();
    }
  }

public:
  CellCard( const token_list_t& tokens ):Card()
  {
    
    unsigned int idx = 0;
    int material;

    ident = makeint(tokens.at(idx++));
    
    if(tokens.at(1) == "like"){
      throw std::runtime_error("LIKE/BUT cell card syntax not yet supported.");
    }

    material = makeint(tokens.at(idx++));
    if(material != 0){
      makedouble(tokens.at(idx++)); // material density, currently ignored.
    }

    token_list_t temp_geom;
    
    // while the tokens appear in geometry-specification syntax, store them into temporary list
    while(idx < tokens.size() && tokens.at(idx).find_first_of("1234567890:#-+()") == 0){
      temp_geom.push_back(tokens[idx++]);
    }

    // retokenize the geometry list, which follows a specialized syntax.
    retokenize_geometry( temp_geom );
    shunt_geometry();

    // store the rest of the tokens into the data list
    while(idx < tokens.size()){
      data.push_back(tokens[idx++]);
    }

  }

  int getIdent() const { 
    return ident;
  }

  void print( std::ostream& s ) const{
    s << "Cell " << ident << " geom " << geom << std::endl;
  }

  iBase_EntityHandle define( iGeom_Instance& geom, InputDeck& data, double universe_size);

};

std::ostream& operator<<(std::ostream& str, const CellCard::geom_list_entry_t& t ){
  switch(t.first){
  case CellCard::LPAREN: str << "("; break;
  case CellCard::RPAREN: str << ")"; break;
  case CellCard::COMPLEMENT: str << "#"; break;
  case CellCard::UNION: str << ":"; break;
  case CellCard::INTERSECT: str << "*"; break;
  case CellCard::SURFNUM: str << t.second; break;
  case CellCard::CELLNUM: str << "c" << t.second; break;
  }
  return str;
}

class AbstractSurface{

public:
  virtual ~AbstractSurface(){}
  
  virtual double getFarthestExtentFromOrigin( ) const = 0;
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size ) = 0;

};

class SurfaceCard : public Card {

protected:
  int ident, coord_xform;
  std::string mnemonic;
  std::vector<double> args;
  AbstractSurface* surface;

public:
  SurfaceCard( const token_list_t tokens ) : Card(), surface(NULL)
  {

    size_t idx = 0;
    std::string token1 = tokens.at(idx++);
    if(token1.find_first_of("*+") != token1.npos){
      std::cerr << "Warning: no special handling for reflecting or white-boundary surfaces" << std::endl;
    token1[0] = ' ';
    }
    ident = makeint(token1);

    std::string token2 = tokens.at(idx++);
    if(token2.find_first_of("1234567890-") != 0){
      //token2 is the mnemonic
      coord_xform = 0;
      mnemonic = token2;
    }
    else{
      // token2 is a coordinate transform identifier
      coord_xform = makeint(token2);
      if(coord_xform < 0){
	throw std::runtime_error("Cannot handle periodic surfaces");
      }
      mnemonic = tokens.at(idx++);
      
    }

    while( idx < tokens.size() ){
      args.push_back( makedouble(tokens[idx++]) );
    }

  }

  ~SurfaceCard(){
    if(surface){
      delete surface;
    }
  }

  int getIdent() const { return ident; } 
					     
  void print( std::ostream& s ) const {
    s << "Surface " << ident << " " << mnemonic << args;
    if( coord_xform != 0 ) s << " TR" << coord_xform;
    s << std::endl;
  }

  AbstractSurface& getSurface();


};



class InputDeck{

public:
  typedef std::vector< CellCard* > cell_card_list;
  typedef std::vector< SurfaceCard* > surface_card_list;

protected:
  cell_card_list cells;
  surface_card_list surfaces;

  std::map<int, CellCard*> cell_map;
  std::map<int, SurfaceCard*> surface_map;

  void parseCells( LineExtractor& lines );
  void parseSurfaces( LineExtractor& lines );

public:

  ~InputDeck(){
    for(cell_card_list::iterator i = cells.begin(); i!=cells.end(); ++i){
      delete *i;
    }
    cells.clear();
    for(surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
      delete *i;
    }
    surfaces.clear();
  }

  cell_card_list& getCells() { return cells; }
  surface_card_list& getSurfaces() { return surfaces; } 

  CellCard* lookup_cell_card(int ident){
    return (*cell_map.find(ident)).second;
  }

  SurfaceCard* lookup_surface_card(int ident){
    return (*surface_map.find(ident)).second;
  }

  static InputDeck& build( std::istream& input );
  
  void createGeometry();


};

class LineExtractor{

protected:
  std::istream& input;
  std::string next_line;
  bool has_next;
  int next_line_idx;

  void get_next(){

    do{
      if(!std::getline(input, next_line)){
	has_next = false;
      }
      else{
	
	next_line_idx++;
	
	// strip trailing carriage return, if any
	if(*(next_line.rbegin()) == '\r')
	  next_line.resize(next_line.size()-1);
	
	// convert to lowercase
	strlower(next_line);
	
      }
    }
    while( has_next && (next_line == "c" || next_line.find("c ") < 5)); 
    // iterate until next_line is not a comment line.
    // although the one-char line "c" does not technically conform to the spec given in the manual,
    // ("a C anywhere in columns 1−5 followed by at least one blank"), I have seen
    // it in practice.

  }

public:
  LineExtractor( std::istream& input_p ) : 
    input(input_p), next_line("*NO INPUT*"), has_next(true), next_line_idx(0)
  {
    get_next();
  }
  
  const std::string& peekLine() const {
    if( has_next ) return next_line;
    else throw std::runtime_error("LineExtractor out of lines, cannot takeLine().");
  }

  const std::string& peekLine( int& lineno ) const {
    lineno = next_line_idx;
    return peekLine();
  }

  std::string takeLine() { 
    if( has_next ){
      std::string ret = next_line;
      get_next();
      return ret;
    }
    else throw std::runtime_error("LineExtractor out of lines, cannot peekLine().");
  }

  std::string takeLine( int& lineno ){
    lineno = next_line_idx;
    return takeLine();
  }

  bool hasLine() const{
    return has_next;
  }

  /**
   * @return the file line number of the next line (as will be returned by either
   * peekLine or takeLine) 
   */
  int getLineCount() const{
    return next_line_idx;
  }

};


void tokenizeLine( std::string line, token_list_t& tokens, const char* extra_separators = "" ){
  
  // replace any occurances of the characters in extra_separators with spaces;
  // they will then act as token separators
  size_t found;
   
  found = line.find_first_of(extra_separators);
  while (found!= line.npos )
  {
    line[found] = ' ';
    found = line.find_first_of(extra_separators,found+1);
  }
  
  std::stringstream str(line);
  while(str){
    std::string t;
    str >> t;

    // skip over $-style inline comments
    size_t idx;
    if((idx = t.find("$")) != t.npos){
      if(idx > 0){
	// this token had some data before the $
	t.resize(idx);
	tokens.push_back(t);
      }
      break;
    }

    // if the token is nontrivial, save it
    // necessary because stringstream may return a "" at the end of lines.
    if(t.length() > 0)  tokens.push_back(t);
  }

}

void parseTitle( LineExtractor& lines ){
  
  // FIXME: will break if the title line looks like a comment card.

  int lineno;
  std::string topLine = lines.takeLine(lineno);
  if(topLine.find("message:") == 0){
    std::clog << "Skipping message block..." << std::endl;
    do{
      // nothing
    }
    while( !isblank(lines.takeLine()) );
	  
    topLine = lines.takeLine(lineno);
  }

  if(topLine.find("continue") == 0){
    std::cerr << "Warning: this looks like it might be a `continue-run' input file." << std::endl;
    std::cerr << "  beware of trouble ahead!" << std::endl;
  }

  std::clog << "The title card is:" << topLine << std::endl;
  std::clog << "    and occupies line " << lineno << std::endl;
}

void InputDeck::parseCells( LineExtractor& lines ){

  std::string line;
  token_list_t token_buffer;

  while( !isblank(line = lines.takeLine()) ){

    tokenizeLine(line, token_buffer, "=");
    std::cout << token_buffer << std::endl;
    
    if( lines.peekLine().find("     ") == 0){
      continue;
    }
    else if( token_buffer.at(token_buffer.size()-1) == "&" ){
      token_buffer.pop_back();
      continue;
    }

    CellCard* c = new CellCard(token_buffer);
    c->print(std::cout);
    this->cells.push_back(c);
    this->cell_map.insert( std::make_pair(c->getIdent(), c) );

    token_buffer.clear();

  }

}

void InputDeck::parseSurfaces( LineExtractor& lines ){
  std::string line;
  token_list_t token_buffer;

  while( !isblank(line = lines.takeLine()) ){

    tokenizeLine(line, token_buffer );
    
    if( lines.peekLine().find("     ") == 0){
      continue;
    }
    else if( token_buffer.at(token_buffer.size()-1) == "&" ){
      token_buffer.pop_back();
      continue;
    }

    SurfaceCard* s = new SurfaceCard(token_buffer);
    s->print(std::cout);
    this->surfaces.push_back(s);
    this->surface_map.insert( std::make_pair(s->getIdent(), s) );

    token_buffer.clear();

  }
}

void parseDataCards( LineExtractor&  ){
}

InputDeck& InputDeck::build( std::istream& input){
 
  LineExtractor lines(input);

  InputDeck* deck = new InputDeck();

  parseTitle(lines);
  deck->parseCells(lines);
  deck->parseSurfaces(lines);
  parseDataCards(lines);

  while(lines.hasLine()){ lines.takeLine(); }
  std::cout << "(total lines: " << lines.getLineCount() << ")" <<  std::endl;

  return *deck;
}


class Vector3d{

public:

  double v[3];
  Vector3d(){
    v[2] = v[1] = v[0] = 0;
  }
  
  Vector3d( const double p[3] ){
    v[0] = p[0];
    v[1] = p[1];
    v[2] = p[2];
  }

  Vector3d( double x, double y, double z ){
    v[0] = x; 
    v[1] = y;
    v[2] = z;
  }

  Vector3d( const std::vector<double>& p ){
    v[0] = p.at(0);
    v[1] = p.at(1);
    v[2] = p.at(2);
  }

  double length() const{
    return sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
  }

};

static Vector3d origin(0,0,0);

#define CHECK_IGEOM(err, msg) \
  do{if((err) != iBase_SUCCESS) std::cerr << "iGeom error (" << err << "): " << msg << std::endl; }while(0)

iBase_EntityHandle makeUniverseSphere( iGeom_Instance& igm, double universe_size ){
  iBase_EntityHandle universe_sphere;
  int igm_result;
  iGeom_createSphere( igm, universe_size, &universe_sphere, &igm_result);
  CHECK_IGEOM( igm_result, "making universe sphere" );
  return universe_sphere;
}

class PlaneSurface : public AbstractSurface { 

public:
  Vector3d normal;
  double offset;

public:
  PlaneSurface( const Vector3d& normal_p, double offset_p ) :
    AbstractSurface(), normal(normal_p), offset(offset_p) 
  {}
  
  virtual double getFarthestExtentFromOrigin() const{
    // this is a funny situation, since planes are technically infinte...
    // in order to have a sane answer, we just return the offset from the origin.
    // (multiplied by root 3, which was done in the old converter, why?)
    return /*sqrt(3.0) * */ std::abs(offset);
  }

  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size){

    int igm_result;
    iBase_EntityHandle universe_sphere = makeUniverseSphere(igm, universe_size);
    iBase_EntityHandle hemisphere;
    iGeom_sectionEnt( igm, &universe_sphere, 
		      normal.v[0], normal.v[1], normal.v[2], offset, !positive, &hemisphere, &igm_result);
    CHECK_IGEOM( igm_result, "Sectioning universe for a plane" );
    return hemisphere;


  }

};

class CylinderSurface : public AbstractSurface {

public:
  enum axis{ X=0, Y=1, Z=2 } axis;
  double radius;
  Vector3d center;
  bool onaxis;

  CylinderSurface( enum axis axis_p, double radius_p ):
    AbstractSurface(), axis(axis_p), radius(radius_p), center(origin), onaxis(true)
  {}

  CylinderSurface( enum axis axis_p, double radius_p, double trans1, double trans2 ):
    AbstractSurface(), axis(axis_p), radius(radius_p), center(origin), onaxis(false)
  {
    switch(axis){
    case X: center.v[Y] += trans1; center.v[Z] += trans2; break;
    case Y: center.v[X] += trans1; center.v[Z] += trans2; break;
    case Z: center.v[X] += trans1; center.v[Y] += trans2; break;
    }
  }
  
  virtual double getFarthestExtentFromOrigin( ) const{
    return radius + center.length();
  }

  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size ){
    int igm_result;

    iBase_EntityHandle cylinder;
    iGeom_createCylinder( igm, 2.0 * universe_size, radius, 0, &cylinder, &igm_result);
    CHECK_IGEOM( igm_result, "making cylinder" );

    
    if( axis == X ){
      iGeom_rotateEnt( igm, &cylinder, 90, 0, 1, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cylinder (X)" );
    }
    else if( axis == Y ){
      iGeom_rotateEnt( igm, &cylinder, 90, 1, 0, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cylinder (Y)" );
    }

    if( onaxis == false ){
      iGeom_moveEnt( igm, &cylinder, center.v[0], center.v[1], center.v[2], &igm_result);
      CHECK_IGEOM( igm_result, "moving cylinder" );
    }

    iBase_EntityHandle universe_sphere = makeUniverseSphere( igm, universe_size );
    iBase_EntityHandle final_cylinder;

    if( positive ){
      iGeom_intersectEnts( igm, universe_sphere, cylinder, &final_cylinder, &igm_result);
      CHECK_IGEOM( igm_result, "making clipped cylinder" );
    }
    else{
      iGeom_subtractEnts( igm, universe_sphere, cylinder, &final_cylinder, &igm_result);
      CHECK_IGEOM( igm_result, "making negative cylinder" );
    }

    return final_cylinder;
  };

};

class SphereSurface : public AbstractSurface {

public:
  Vector3d center;
  double radius;

  iBase_EntityHandle positiveSense_volume;
  iBase_EntityHandle negativeSense_volume;

public:
  SphereSurface( const Vector3d& center_p, double radius_p ) :
    AbstractSurface(), center(center_p), radius(radius_p),
    positiveSense_volume(NULL), negativeSense_volume(NULL)
  {}

  virtual ~SphereSurface(){}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return (center.length() + radius);
  }

  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size ){

    int igm_result;
    if(positive){
      //if(!positiveSense_volume){
	
	iGeom_createSphere( igm, radius, &positiveSense_volume, &igm_result);
	CHECK_IGEOM( igm_result, "making sphere" );
	
	iGeom_moveEnt( igm, &positiveSense_volume, center.v[0], center.v[1], center.v[2], &igm_result);
	CHECK_IGEOM( igm_result, "moving sphere" );

	//} 
      std::cout << "Returning " << positiveSense_volume << std::endl;
      return positiveSense_volume;
    }
    else{ // negative sense 

      //if(!negativeSense_volume){

	// first ensure that positiveSense_volume is set
	this->getHandle( true, igm, universe_size );

	iBase_EntityHandle universe_sphere = makeUniverseSphere(igm, universe_size);

	iGeom_subtractEnts( igm, universe_sphere, positiveSense_volume, &negativeSense_volume, &igm_result);
	CHECK_IGEOM( igm_result, "subtracting sphere" );
	//}

      return negativeSense_volume;

    }
  }

};

AbstractSurface& SurfaceCard::getSurface() {
  // SurfaceCard variables: surface, mnemonic, args
  

  if(!this->surface){
    if( mnemonic == "so"){
      this->surface = new SphereSurface( origin, args.at(0) );
    }
    else if( mnemonic == "sx"){
      this->surface = new SphereSurface( Vector3d( args.at(0), 0, 0 ), args.at(1) );
    }
    else if( mnemonic == "sy"){
      this->surface = new SphereSurface( Vector3d( 0, args.at(0), 0 ), args.at(1) );
    }
    else if( mnemonic == "sz"){
      this->surface = new SphereSurface( Vector3d( 0, 0, args.at(0) ), args.at(1) );
    }
    else if( mnemonic == "s" || mnemonic == "sph" ){
      this->surface = new SphereSurface( Vector3d( args ), args.at(3) );
    }
    else if( mnemonic == "p"){
      this->surface = new PlaneSurface( Vector3d( args ), args.at(3) );
    }
    else if( mnemonic == "px"){
      this->surface = new PlaneSurface( Vector3d( 1, 0, 0), args.at(0) );
    }
    else if( mnemonic == "py"){
      this->surface = new PlaneSurface( Vector3d( 0, 1, 0), args.at(0) );
    }
    else if( mnemonic == "pz"){
      this->surface = new PlaneSurface( Vector3d( 0, 0, 1), args.at(0) );
    }
    else if( mnemonic == "cx" ){
      this->surface = new CylinderSurface( CylinderSurface::X, args.at(0) );
    }
    else if( mnemonic == "cy" ){
      this->surface = new CylinderSurface( CylinderSurface::Y, args.at(0) );
    }
    else if( mnemonic == "cz" ){
      this->surface = new CylinderSurface( CylinderSurface::Z, args.at(0) );
    }
    else if( mnemonic == "c/x"){
      this->surface = new CylinderSurface( CylinderSurface::X, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/y"){
      this->surface = new CylinderSurface( CylinderSurface::Y, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/z"){
      this->surface = new CylinderSurface( CylinderSurface::Z, args.at(2), args.at(0), args.at(1) );
    }
    else{
      throw std::runtime_error( mnemonic + " is not a supported surface" );
    }
  }

  return *(this->surface);
}


iBase_EntityHandle CellCard::define( iGeom_Instance& igm, InputDeck& data, double universe_size){

  int igm_result;

  std::vector<iBase_EntityHandle> stack;
  for(geom_list_t::iterator i = this->geom.begin(); i!=geom.end(); ++i){
    
    geom_list_entry_t& token = (*i);
    switch(token.first){
    case CELLNUM:
      stack.push_back(data.lookup_cell_card(token.second)->define(igm,data,universe_size));
      break;
    case SURFNUM:
      {      
	int surface = token.second;
	bool pos = true;
	if( surface < 0){
	  pos = false; surface = -surface;
	}
	try{
	  AbstractSurface& surf = data.lookup_surface_card( surface )->getSurface();
	  iBase_EntityHandle surf_handle = surf.getHandle( pos, igm, universe_size );
	  stack.push_back(surf_handle);
	}
	catch(std::runtime_error& e) { std::cerr << e.what() << std::endl; }
      }
      break;
    case INTERSECT:
      {
	assert( stack.size() >= 2 );
	iBase_EntityHandle s1 = stack.back(); stack.pop_back();
	iBase_EntityHandle s2 = stack.back(); stack.pop_back();
	iBase_EntityHandle result;
	iGeom_intersectEnts( igm, s1, s2, &result, &igm_result);
	CHECK_IGEOM( igm_result, "Intersecting two entities" );
	stack.push_back(result);
      }
      break;
    case UNION:
      {	
	assert( stack.size() >= 2 );
	iBase_EntityHandle s[2];
	s[0] = stack.back(); stack.pop_back();
	s[1] = stack.back(); stack.pop_back();
	iBase_EntityHandle result;
	iGeom_uniteEnts( igm, s, 2, &result, &igm_result);
	CHECK_IGEOM( igm_result, "Uniting two entities" );
	stack.push_back(result);
      }
      break;
    case COMPLEMENT:
      {
	assert (stack.size() >= 1 );
      	iBase_EntityHandle universe_sphere = makeUniverseSphere(igm, universe_size);
	iBase_EntityHandle s = stack.back(); stack.pop_back();
	iBase_EntityHandle result;

	iGeom_subtractEnts( igm, universe_sphere, s, &result, &igm_result);
	CHECK_IGEOM( igm_result, "Complementing an entity" );
	stack.push_back(result);
      }
      break;
    default:
      throw std::runtime_error( "Unexpected token while evaluating cell geometry");
      break;
    }
  }

  assert( stack.size() == 1);
  return stack[0];

}



void InputDeck::createGeometry(){

  iGeom_Instance igm;
  int igm_result;
  
  iGeom_newGeom( 0, &igm, &igm_result, 0);
  CHECK_IGEOM( igm_result, "Initializing iGeom");

  double universe_size = 0;
  for( surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
    try{
    universe_size = std::max( universe_size, (*i)->getSurface().getFarthestExtentFromOrigin() );
    } catch(std::runtime_error& e){}
  }
  universe_size *= 1.2;

  std::cout << "Universe size: " << universe_size << std::endl;
	
  for( cell_card_list::iterator i = cells.begin(); i!=cells.end(); ++i){
    (*i)->define( igm, *this, universe_size );
  }

  std::string outName = "out.sat";
  iGeom_save( igm, outName.c_str(), "", &igm_result, outName.length(), 0 );
  CHECK_IGEOM( igm_result, "saving the output file "+outName );

}

  
int main(int argc, char* argv[]){

  std::string input_file = "INP";
  if(argc > 1){ input_file = argv[1]; }

  std::ifstream input(input_file.c_str(), std::ios::in );
    InputDeck& d = InputDeck::build(input);

    InputDeck::surface_card_list& surfaces = d.getSurfaces();
    for( InputDeck::surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
      try{
	const AbstractSurface& s = (*i)->getSurface();
	std::cout << "Distance from origin: " << s.getFarthestExtentFromOrigin() << std::endl;
      }
      catch(std::runtime_error& e){
	std::cout << e.what() << std::endl;
      }
    }

    d.createGeometry();

    return 0;
    

    int igm_result;
    iGeom_Instance igm;
    iGeom_newGeom( 0, &igm, &igm_result, 0);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error initializing iGeom" << std::endl;
    }

    /**\brief Create a sphere centered on the origin
     */
    iBase_EntityHandle sphere;
    iGeom_createSphere( igm, 150, &sphere, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error making sphere" << std::endl;
    }

    std::cout << "s1" << std::endl;

    iBase_EntityHandle sphere2;
    iGeom_createSphere( igm, 35, &sphere2, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error making sphere2" << std::endl;
    }

    std::cout << "s2" << std::endl;

    iGeom_moveEnt( igm, &sphere2, 150, 0, 0, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error moving sphere2" << std::endl;
    }

    std::cout << "move" << std::endl;

    iBase_EntityHandle intersection;
    iGeom_subtractEnts( igm, sphere, sphere2, &intersection, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error doing intersection" << std::endl;
    }
    
    std::cout << "intersect" << std::endl;

    iBase_EntityHandle final;
    iGeom_sectionEnt( igm, &intersection, 0, 1, 0, 0, false, &final, &igm_result);

    std::string outName = "out.sat";
    iGeom_save( igm, outName.c_str(), "", &igm_result, outName.length(), 0 );
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error saving iGeom" << std::endl;
    }

}
