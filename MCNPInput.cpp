#include "MCNPInput.hpp"

#include <stdexcept>
#include <cassert>
#include <iostream>
#include <sstream>

/******************
 * HELPER FUNCTIONS
 ******************/

static std::string& strlower( std::string& str ){
  // convert to lowercase
  for(size_t i = 0; i < str.length(); ++i){
    str[i] = tolower( str.at(i) );
  }
  return str;
}


static int makeint( const std::string& token ){
  const char* str = token.c_str();
  char* end;
  int ret = strtol(str, &end, 10);
  if( end != str+token.length() ){
    std::cerr << "Warning: string [" << token << "] did not convert to int as expected." << std::endl;
  }
  return ret;
}

static double makedouble( const std::string& token ){
  const char* str = token.c_str();
  char* end;
  double ret = strtod(str, &end);
  if( end != str+token.length() ){
    std::cerr << "Warning: string [" << token << "] did not convert to double as expected." << std::endl;
  }
  return ret;
}


static bool isblank( const std::string& line ){
  return (line=="" || line.find_first_not_of(" ") == line.npos );
}


template < class T >
std::ostream& operator<<( std::ostream& out, const std::vector<T>& list ){

  out << "[";

  for(typename std::vector<T>::const_iterator i = list.begin(); i!=list.end(); ++i){
    out << *i << "|";
  }

  if(list.size() > 0) 
    out << "\b"; // unless list was empty, backspace the last | character

  out << "]" << std::endl;
  return out;
}


/******************
 * CELL CARDS
 ******************/

class CellCardImpl : public CellCard { 

protected:
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
  CellCardImpl( InputDeck& deck, const token_list_t& tokens ) : 
    CellCard( deck )
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
};

CellCard::CellCard( InputDeck& deck ) :
  Card(deck)
{}

void CellCard::print( std::ostream& s ) const {
    s << "Cell " << ident << " geom " << geom << std::endl;
}

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



/******************
 * SURFACE CARDS
 ******************/

SurfaceCard::SurfaceCard( InputDeck& deck, const token_list_t tokens ):
  Card(deck)
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
	// (-coord_xform) is the ID of surface with respect to which this surface is periodic.
	throw std::runtime_error("Cannot handle periodic surfaces");
      }
      mnemonic = tokens.at(idx++);
      
    }

    while( idx < tokens.size() ){
      args.push_back( makedouble(tokens[idx++]) );
    }

}

void SurfaceCard::print( std::ostream& s ) const {
  s << "Surface " << ident << " " << mnemonic << args;
  if( coord_xform != 0 ) s << " TR" << coord_xform;
  s << std::endl;
}


/******************
 * DATA CARDS
 ******************/

TransformCard::TransformCard( InputDeck& deck, int ident_p, bool degree_format, const token_list_t& input ):
  DataCard(deck), ident(ident_p)
{
  std::vector<double> args;
  for( token_list_t::const_iterator i = input.begin(); i!=input.end(); ++i){
    args.push_back( makedouble( *i ) );
  }
  trans = Transform( args, degree_format );
}

void TransformCard::print( std::ostream& str ){
  str << "TR" << ident << ": ";
  //trans.print(str);
  str << std::endl;
}


/******************
 * PARSING UTILITIES 
 ******************/

class InputDeck::LineExtractor{

protected:
  std::istream& input;
  std::string next_line;
  bool has_next;
  int next_line_idx;

  /* 
   * Take note of the following, from mcnp5 manual page 1-3:
   * Tab characters in the input file are converted to one or more blanks, such that the character
   * following the tab will be positioned at the next tab stop. Tab stops are set every 8 characters,
   * i.e., 9, 17, 25, etc. The limit of input lines to 80 columns applies after tabs are expanded into blank
   * spaces. </snip>
   * I don't know whether this needs to be addressed to handle corner cases for line continuation, etc.
   * currently, it is not addressed.
   */

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
    else throw std::runtime_error("LineExtractor out of lines, cannot peekLine().");
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
    else throw std::runtime_error("LineExtractor out of lines, cannot takeLine().");
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

/******************
 * INPUT DECK
 ******************/

InputDeck::~InputDeck(){
  for(cell_card_list::iterator i = cells.begin(); i!=cells.end(); ++i){
    delete *i;
  }
  cells.clear();
  for(surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
    delete *i;
  }
  surfaces.clear();
  for(data_card_list::iterator i = datacards.begin(); i!=datacards.end(); ++i){
    delete *i;
  }
  datacards.clear();
  
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

    CellCard* c = new CellCardImpl(*this, token_buffer);
    c->print(std::cout);
    this->cells.push_back(c);
    this->cell_map.insert( std::make_pair(c->getIdent(), c) );

    token_buffer.clear();

  }

}



void InputDeck::parseTitle( LineExtractor& lines ){
  
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

    SurfaceCard* s = new SurfaceCard(*this, token_buffer);
    s->print(std::cout);
    this->surfaces.push_back(s);
    this->surface_map.insert( std::make_pair(s->getIdent(), s) );

    token_buffer.clear();

  }
}

void InputDeck::parseDataCards( LineExtractor& lines ){

  std::string line;
  token_list_t token_buffer;

  while( lines.hasLine() && !isblank(line = lines.takeLine()) ){

    tokenizeLine(line, token_buffer );
    
    if( lines.hasLine() && lines.peekLine().find("     ") == 0){
      continue;
    }
    else if( token_buffer.at(token_buffer.size()-1) == "&" ){
      token_buffer.pop_back();
      continue;
    }
    else if( token_buffer.at(0) == "#" ){
      std::cerr << "Vertical data card format not supported" << std::endl;
      std::cerr << "Data written in this format will be ignored." << std::endl;
    }

    DataCard* d = NULL;
    DataCard::kind t = DataCard::OTHER;
    int ident = 0;

    std::string cardname = token_buffer.at(0);
    token_buffer.erase( token_buffer.begin() );

    if( cardname.find("tr") == 0 ){
      t = DataCard::TR;
      std::string id_string( cardname, 2 );
      bool degree_format = false;
      if(id_string.at(id_string.length()-1) == '*'){ // last char is a *
	degree_format = true;
	id_string.resize(id_string.length()-1);
      }
      ident = makeint( id_string );
      d = new TransformCard( *this, ident, degree_format, token_buffer);
    }
    
    if(d){
      d->print( std::cout );
      this->datacards.push_back(d);
      this->datacard_map.insert( std::make_pair( std::make_pair(t,ident), d) );
    }

    token_buffer.clear();

  }

}

InputDeck& InputDeck::build( std::istream& input){
 
  LineExtractor lines(input);

  InputDeck* deck = new InputDeck();

  deck->parseTitle(lines);
  deck->parseCells(lines);
  deck->parseSurfaces(lines);
  deck->parseDataCards(lines);

  while(lines.hasLine()){ lines.takeLine(); }
  std::cout << "(total lines: " << lines.getLineCount() << ")" <<  std::endl;

  return *deck;
}


