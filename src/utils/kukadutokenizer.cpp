///////////////////////////////////////////////////////////////////////////////
// KukaduTokenizer.cpp
// =============
// General purpose string KukaduTokenizer (C++ string version)
//
// The default delimiters are space(" "), tab(\t, \v), newline(\n),
// carriage return(\r), and form feed(\f).
// If you want to use different delimiters, then use setDelimiter() to override
// the delimiters. Note that the delimiter string can hold multiple characters.
//
//  AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// CREATED: 2005-05-25
// UPDATED: 2011-03-08
///////////////////////////////////////////////////////////////////////////////

#include <kukadu/utils/kukadutokenizer.hpp>
#include <iostream>

using namespace std;

namespace kukadu {

    ///////////////////////////////////////////////////////////////////////////////
    // constructor
    ///////////////////////////////////////////////////////////////////////////////
    KukaduTokenizer::KukaduTokenizer() : buffer(""), token(""), delimiter(DEFAULT_DELIMITER) {
        currPos = 0;
        lastToken = "";
        useLastToken = false;
        tokenIdx = -1;
    }

    KukaduTokenizer::KukaduTokenizer(const std::string& str, const std::string& delimiter) : buffer(str), token(""), delimiter(delimiter) {
        currPos = 0;
        lastToken = "";
        useLastToken = false;
        tokenIdx = -1;
    }



    ///////////////////////////////////////////////////////////////////////////////
    // destructor
    ///////////////////////////////////////////////////////////////////////////////
    KukaduTokenizer::~KukaduTokenizer() {
    }



    ///////////////////////////////////////////////////////////////////////////////
    // reset string buffer, delimiter and the currsor position
    ///////////////////////////////////////////////////////////////////////////////
    void KukaduTokenizer::set(const std::string& str, const std::string& delimiter) {
        this->buffer = str;
        this->delimiter = delimiter;
        this->currPos = 0;
    }

    void KukaduTokenizer::setString(const std::string& str) {
        this->buffer = str;
        this->currPos = 0;
    }

    void KukaduTokenizer::setDelimiter(const std::string& delimiter) {
        this->delimiter = delimiter;
        this->currPos = 0;
    }

    int KukaduTokenizer::getTokenIdx() {
        return tokenIdx;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // return the next token
    // If cannot find a token anymore, return "".
    ///////////////////////////////////////////////////////////////////////////////
    std::string KukaduTokenizer::next() {
        ++tokenIdx;
        if(useLastToken) {
            useLastToken = false;
            return lastToken;
        } else {
            if(buffer.size() <= 0) return "";           // skip if buffer is empty

            token.clear();                              // reset token string

            this->skipDelimiter();                      // skip leading delimiters

            // append each char to token string until it meets delimiter
            while(currPos != buffer.size() && !isDelimiter(*(buffer.begin() + currPos))) {
                token += *(buffer.begin() + currPos);
                ++currPos;
            }
            return (lastToken = token);
        }
        return "";
    }

    void KukaduTokenizer::putBackLast() {
        useLastToken = true;
        if(tokenIdx > 0)
            tokenIdx--;
    }



    ///////////////////////////////////////////////////////////////////////////////
    // skip ang leading delimiters
    ///////////////////////////////////////////////////////////////////////////////
    void KukaduTokenizer::skipDelimiter() {
        while(currPos != buffer.size() && isDelimiter(*(buffer.begin() + currPos)))
            ++currPos;
    }



    ///////////////////////////////////////////////////////////////////////////////
    // return true if the current character is delimiter
    ///////////////////////////////////////////////////////////////////////////////
    bool KukaduTokenizer::isDelimiter(char c) {
        return (delimiter.find(c) != std::string::npos);
    }



    ///////////////////////////////////////////////////////////////////////////////
    // split the input string into multiple tokens
    // This function scans tokens from the current cursor position.
    ///////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> KukaduTokenizer::split() {
        std::vector<std::string> tokens;
        std::string token;
        while((token = this->next()) != "") {
            tokens.push_back(token);
        }

        return tokens;
    }

}
